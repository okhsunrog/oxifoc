mod model;
mod parameters;
mod plot;
mod viewport;
use dioxus::html::geometry::WheelDelta;
use dioxus::prelude::*;
use dioxus_native::{CustomWidgetAttr, LogicalSize, WindowAttributes};
use model::{CAPACITY, Model, Snapshot};
use oxifoc_core::types::ControlMode;
use oxifoc_core::units::amps;
use std::{
    sync::{Arc, Mutex, atomic::Ordering},
    time::Duration,
};
use viewport::Viewport;
const CSS: &str = include_str!("style.css");
fn main() {
    oxifoc_host_lib::init_tracing();
    if std::env::args().any(|a| a == "--verify-virtual") {
        model::verify_virtual().unwrap();
        return;
    }
    let mut window = WindowAttributes::default().with_window_icon(Some(
        dioxus_native::winit::icon::RgbaIcon::new(
            include_bytes!("../../oxifoc-ui/assets/oxifoc-icon-64.rgba").to_vec(),
            64,
            64,
        )
        .expect("embedded icon dimensions")
        .into(),
    ));
    #[cfg(target_os = "linux")]
    {
        use dioxus_native::winit::platform::wayland::WindowAttributesWayland;
        window = window.with_platform_attributes(Box::new(
            WindowAttributesWayland::default().with_name("com.oxifoc.studio.native", "oxifoc"),
        ));
    }
    dioxus_native::launch_cfg(
        App,
        vec![],
        vec![
            Box::new(
                window
                    .with_title("OxiFOC • Drive studio")
                    .with_surface_size(if std::env::args().any(|s| s == "--compact") {
                        LogicalSize::new(420, 900)
                    } else {
                        LogicalSize::new(1440, 1000)
                    }),
            ),
            Box::new(oxifoc_plot::required_features()),
            Box::new(oxifoc_plot::required_limits(CAPACITY, 5)),
        ],
    );
}
#[component]
fn App() -> Element {
    if std::env::args().any(|s| s == "--catalogue") {
        return rsx! { oxifoc_ui::Catalogue {} };
    }
    let flow = oxifoc_host_dioxus_ui::use_connection(oxifoc_ui::Capabilities::DESKTOP);
    let session = (flow.snapshot)().session;
    match session {
        Some(session) => {
            rsx! { Studio { session, ondisconnect: move |_| flow.controller.disconnect() } }
        }
        None => rsx! { oxifoc_host_dioxus_ui::ConnectScreen { flow } },
    }
}

#[component]
fn Studio(
    session: oxifoc_host_dioxus_ui::ConnectedSession,
    ondisconnect: EventHandler<()>,
) -> Element {
    let model = use_hook(|| Model::from_runtime(session.runtime.clone()));
    let mut data = use_signal(Snapshot::default);
    let mut frame = use_signal(|| 0u64);
    let mut page = use_signal(|| 0);
    let editor = use_signal(parameters::Editor::default);
    let mut paused = use_signal(|| false);
    let mut current = use_signal(|| "1.0".to_string());
    let mut error = use_signal(String::new);
    let m = model.clone();
    use_future(move || {
        let m = m.clone();
        async move {
            loop {
                tokio::time::sleep(Duration::from_millis(100)).await;
                data.set(m.snapshot.lock().unwrap().clone());
                frame += 1;
            }
        }
    });
    let m = model.clone();
    use_drop(move || {
        m.alive.store(false, Ordering::Relaxed);
    });
    let m = model.clone();
    use_effect(move || {
        let s = data.read();
        parameters::sync_connection(
            editor,
            &m,
            s.connected.then_some(s.connection_generation),
            page() == 1,
        );
    });
    let s = data();
    let online = s.connected;
    let connection = online.then_some(s.connection_generation);
    let busy = s.pending;
    let endpoint = format!(
        "{}  {}",
        session.form.transport.label(),
        session.form.endpoint()
    );
    let rpm = if online {
        format!("{:.0}", s.rpm)
    } else {
        "—".into()
    };
    let volts = if online {
        format!("{:.1}", s.volts)
    } else {
        "—".into()
    };
    let temp = if online {
        format!("{:.1}", s.temperature)
    } else {
        "—".into()
    };
    let rate = format!("{:.0}", s.hz);
    let conn = if online {
        "Connected"
    } else {
        "Waiting for controller"
    };
    let state = if online {
        s.state.clone()
    } else {
        "Offline".into()
    };
    let pmodel = model.clone();
    let runmodel = model.clone();
    let stopmodel = model.clone();
    rsx! {
    style {"{oxifoc_ui::CSS}"}
    style {"{CSS}"}
    div {class:if page()==1 {"shell parameter-shell"} else {"shell"},
     aside {class:"rail",
      oxifoc_ui::Brand {}
      p {class:"rail-caption","Drive studio"}
      nav {for (i,label) in ["Oscilloscope","Parameters","Session journal"].iter().enumerate(){
       button {class:if page()==i {"nav selected"} else {"nav"},onclick:move |_|page.set(i),onkeydown:move |e|{if oxifoc_ui::keyboard_activate(&e){page.set(i);}},"{label}"}
      }}
      div {class:"rail-bottom",strong {"Native workspace"} p {"Motor control & diagnostics"} span {"Experimental GUI"}}
     }
     div {class:"body",
      header {div {class:"connection",span {class:if online{"dot online"}else{"dot"}},strong {"{conn}"} span {"{endpoint}"}} span {class:"session-state","{state}"} button {class:"button",onclick:move |_| ondisconnect.call(()),onkeydown:move |e|{if oxifoc_ui::keyboard_activate(&e){ondisconnect.call(());}}, "Disconnect"}}
      div {class:"workspace",
       main {
        if page()==0 {
         oxifoc_ui::Instruments {
          rpm: (online && s.fast_seen.is_some_and(|t| t.elapsed().as_secs_f32() < 2.)).then_some(s.rpm),
          iq: s.iq.filter(|_| online && s.fast_seen.is_some_and(|t| t.elapsed().as_secs_f32() < 2.)),
          fet: (online && s.slow_seen.is_some_and(|t| t.elapsed().as_secs_f32() < 3.)).then_some(s.temperature),
          motor: s.motor_temp.filter(|_| online && s.slow_seen.is_some_and(|t| t.elapsed().as_secs_f32() < 3.)),
         }
         div {class:"titlebar",
          div {
           h1 {"Drive observatory"}
           p {"Currents, motion and operating conditions"}
          }
          button {class:"button",onclick:move |_| {paused.toggle();pmodel.paused.store(paused(),Ordering::Relaxed);},if paused(){"Resume plots"}else{"Pause plots"}}
         }
         if !online {div {class:"notice","Waiting for the selected controller. Retained traces are stale."}}
         div {class:"plot-stack",
          Chart {model:model.clone(),index:0,title:"Phase & dq currents",unit:"A",tick:frame(),paused:paused()}
          div {class:"twoplots",
           Chart {model:model.clone(),index:1,title:"Rotor speed",unit:"RPM",tick:frame(),paused:paused()}
           div {class:"slowplots",
            Chart {model:model.clone(),index:2,title:"DC bus",unit:"V",tick:frame(),paused:paused()}
            Chart {model:model.clone(),index:3,title:"Power stage",unit:"°C",tick:frame(),paused:paused()}
           }
          }
         }
        } else if page()==1 {
         parameters::Parameters {editor, model: model.clone(), connection, running: s.state == "Running"}
        } else {
         div {class:"titlebar",
          div {
           h1 {"Session journal"}
           p {"Connection changes and acknowledged commands"}
          }
         }
         section {class:"journal",if s.events.is_empty(){p {"No session events yet."}} for (i,event) in s.events.iter().enumerate().rev(){div {class:"event",span {"{i}"} p {"{event}"}}}}
        }
       }
       aside {class:"inspector",
        div {class:"inspector-title",span {class:"motor-glyph","◎"} h2 {"Drive control"} p {"Acknowledged controller commands"}}
        div {class:"reading",span {"Rotor speed"} strong {"{rpm}"} small {"RPM"}}
        div {class:"reading-pair",div {span {"DC bus"} strong {"{volts}"} small {" V"}} div {span {"FET temperature"} strong {"{temp}"} small {" °C"}}}
        div {class:"control-group",h3 {"Torque command"} label {r#for:"iq","Target Iq, A"} input {id:"iq",r#type:"text",value:"{current}",oninput:move |e|current.set(e.value())}
         p {"Id = 0 A. Apply sends current-control mode."}
         button {class:"button primary wide",disabled:!online||busy,onclick:move |_| {
          match current().parse::<f32>() {Ok(iq) if iq.is_finite()&&iq.abs()<=10.=>{error.set(String::new());runmodel.motor(ControlMode::CurrentControl{iq_target: amps(iq),id_target: amps(0.)});},_=>error.set("Enter a finite current from −10 to 10 A.".into())}
         },if busy {"Waiting for acknowledgement…"} else {"Apply current"}}
         button {class:"button stop wide",disabled:!online,onclick:move |_|stopmodel.motor(ControlMode::Stopped),"Stop drive"}
         if !error().is_empty(){p {class:"error","{error}"}}
        }
        div {class:"health",h3 {"Acquisition"} div {span {"Received"} strong {"{rate} Hz"}} div {span {"Samples"} strong {"{s.samples}"}} div {span {"Active faults"} strong {"{s.faults}"}} p {if paused(){"Plots paused. Acquisition continues."}else{"Live telemetry from the controller."}}}
       }
      }
      footer {span {"OxiFOC native"} span {"{conn}"} if page()==0 {span {"Wheel to zoom • double-click a plot to focus"}}}
     }
    }
    }
}
#[component]
fn Chart(
    model: Arc<Model>,
    index: usize,
    title: &'static str,
    unit: &'static str,
    tick: u64,
    paused: bool,
) -> Element {
    let view = use_hook(|| {
        Arc::new(Mutex::new(plot::View {
            visible: if index < 2 { 1000 } else { 100 },
            offset: 0,
            width: 1,
            scale: 1.,
            channels: vec![true; if index == 0 { 5 } else { 1 }],
            bounds: (0., 0.),
        }))
    });
    let m = model.clone();
    let v = view.clone();
    let attr = use_hook(move || CustomWidgetAttr::new(plot::Plot::new(m, index, v)));
    let mut focus = use_signal(|| false);
    dioxus_native::use_window_event(move |event, _| {
        use dioxus_native::winit::{
            event::{ElementState, WindowEvent},
            keyboard::{Key, NamedKey},
        };
        if let WindowEvent::KeyboardInput { event, .. } = event
            && event.state == ElementState::Pressed
            && event.logical_key == Key::Named(NamedKey::Escape)
        {
            focus.set(false);
        }
    });
    let mut revision = use_signal(|| 0u64);
    let wheel = view.clone();
    let reset = view.clone();
    let label = {
        let v = view.lock().unwrap();
        format!("{} samples", v.visible)
    };
    if !paused {
        view.lock().unwrap().offset = 0;
    }
    let bounds = view.lock().unwrap().bounds;
    let upper = format!("{:.2}", bounds.1);
    let lower = format!("{:.2}", bounds.0);
    let _ = (tick, revision());
    rsx! {section {class:if focus(){"plot-card focused"}else if index==0{"plot-card primary-plot"}else{"plot-card"},
     div {class:"plot-heading",h2 {"{title}"} span {"{unit}"} button {class:"text-button",onclick:move |_|{let mut v=reset.lock().unwrap();v.visible=if index<2{1000}else{100};v.offset=0;revision+=1;},"Reset"} button {class:"text-button",onclick:move |_|focus.toggle(),if focus(){"Restore"}else{"Focus"}}}
     div {class:"plot-surface",ondoubleclick:move |_|focus.toggle(),onwheel:move |e|{
      e.prevent_default();e.stop_propagation();let delta=match e.delta(){WheelDelta::Pixels(v)=>v.y,WheelDelta::Lines(v)=>v.y*24.,WheelDelta::Pages(v)=>v.y*240.};
      let mut v=wheel.lock().unwrap();let anchor=e.element_coordinates().x/(v.width as f64/v.scale);let available=model.plots[index].available_samples().max(2);
      let next=Viewport{visible:v.visible,offset:v.offset}.zoom(delta,anchor,paused,available.min(CAPACITY as u32));v.visible=next.visible;v.offset=next.offset;revision+=1;
     },object {"data":attr,"data-frame":"{tick}-{revision}"} span {class:"y-upper","{upper}"} span {class:"y-lower","{lower}"}}
     div {class:"plot-caption",if index==0 {for (i,name) in ["Ia","Ib","Ic","Id","Iq"].iter().enumerate(){
      {let v=view.clone();let active=v.lock().unwrap().channels[i];rsx!{button {class:if active{"channel active"}else{"channel"},onclick:move |_|{let mut v=v.lock().unwrap();v.channels[i]=!v.channels[i];revision+=1;},"{name}"}}}
     }} span {"{label}"} span {"Auto Y"}}
    }}
}
