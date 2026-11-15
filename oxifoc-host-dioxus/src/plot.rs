use crate::model::{CAPACITY, Model};
use anyrender::{AntiAliasing, PaintRef, PaintScene, RenderContext, ResourceId, Scene};
use blitz_dom::{Widget, node::ComputedStyles};
use oxifoc_plot::{PlotConfig, PlotRenderer};
use peniko::{
    Fill, ImageBrush, ImageSampler,
    kurbo::{Affine, Rect},
};
use std::sync::{Arc, Mutex};
pub struct View {
    pub visible: u32,
    pub offset: u32,
    pub width: u32,
    pub scale: f64,
    pub channels: Vec<bool>,
    pub bounds: (f32, f32),
}
pub struct Plot {
    pub model: Arc<Model>,
    pub index: usize,
    pub view: Arc<Mutex<View>>,
    renderer: Option<PlotRenderer>,
    resource: Option<(ResourceId, wgpu::Texture)>,
}
impl Plot {
    pub fn new(model: Arc<Model>, index: usize, view: Arc<Mutex<View>>) -> Self {
        Self {
            model,
            index,
            view,
            renderer: None,
            resource: None,
        }
    }
}
impl Widget for Plot {
    fn can_create_surfaces(&mut self, ctx: &mut dyn RenderContext) {
        let h = ctx
            .renderer_specific_context()
            .unwrap()
            .downcast::<wgpu_context::DeviceHandle>()
            .unwrap();
        self.renderer = Some(PlotRenderer::new(
            &h.device,
            &h.queue,
            PlotConfig {
                num_channels: if self.index == 0 { 5 } else { 1 },
                capacity: CAPACITY,
                y_min: if self.index == 0 { -5. } else { 0. },
                y_max: if self.index == 0 { 5. } else { 30. },
                auto_range: true,
                channel_colors: vec![
                    [0.15, 0.36, 0.85, 1.],
                    [0.02, 0.53, 0.48, 1.],
                    [0.85, 0.36, 0.16, 1.],
                    [0.52, 0.32, 0.72, 1.],
                    [0.71, 0.31, 0.48, 1.],
                ]
                .into_iter()
                .take(if self.index == 0 { 5 } else { 1 })
                .collect(),
            },
        ));
    }
    fn destroy_surfaces(&mut self) {
        self.renderer = None;
        self.resource = None;
    }
    fn requires_redraw(&self) -> bool {
        self.model
            .runtime
            .connected
            .load(std::sync::atomic::Ordering::Relaxed)
            && !self.model.paused.load(std::sync::atomic::Ordering::Relaxed)
    }
    fn paint(
        &mut self,
        ctx: &mut dyn RenderContext,
        _: &ComputedStyles,
        width: u32,
        height: u32,
        scale: f64,
    ) -> Scene {
        let mut scene = Scene::new();
        if width == 0 || height == 0 {
            return scene;
        }
        let Some(r) = self.renderer.as_mut() else {
            return scene;
        };
        let mut v = self.view.lock().unwrap();
        v.width = width;
        v.scale = scale;
        r.set_channel_visibility(&v.channels);
        let out = r.render(
            &self.model.plots[self.index],
            width,
            height,
            v.visible,
            v.offset,
            scale as f32,
        );
        v.bounds = (out.y_min, out.y_max);
        if self
            .resource
            .as_ref()
            .is_none_or(|(_, t)| *t != out.texture)
        {
            if let Some((id, _)) = self.resource.take() {
                ctx.unregister_resource(id);
            }
            let id = ctx
                .try_register_custom_resource(Box::new(out.texture.clone()))
                .unwrap();
            self.resource = Some((id, out.texture));
        }
        scene.fill(
            Fill::NonZero,
            Affine::IDENTITY,
            PaintRef::Resource(ImageBrush {
                image: self.resource.as_ref().unwrap().0,
                sampler: ImageSampler::default(),
            }),
            None,
            &Rect::from_origin_size((0., 0.), (width as f64, height as f64)),
            AntiAliasing::Enabled,
        );
        scene
    }
}
