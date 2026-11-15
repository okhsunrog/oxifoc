mod model;
mod plot;
mod viewport;
use dioxus::html::geometry::WheelDelta;
use dioxus::prelude::*;
use dioxus_native::{CustomWidgetAttr, LogicalSize, WindowAttributes};
use model::{CAPACITY, Model, Snapshot};
use oxifoc_core::types::ControlMode;
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
    dioxus_native::launch_cfg(
        App,
        vec![],
        vec![
            Box::new(
                WindowAttributes::default()
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
    let s = data();
    let online = s.connected;
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
    let configmodel = model.clone();
    rsx! {
    style {"{oxifoc_ui::CSS}"}
    style {"{CSS}"}
    div {class:"shell",
     aside {class:"rail",
      oxifoc_ui::Brand {}
      p {class:"rail-caption","Drive studio"}
      nav {for (i,label) in ["Oscilloscope","Parameters","Session journal"].iter().enumerate(){
       button {class:if page()==i {"nav selected"} else {"nav"},onclick:move |_|page.set(i),"{label}"}
      }}
      div {class:"rail-bottom",strong {"Native workspace"} p {"Motor control & diagnostics"} span {"Experimental GUI"}}
     }
     div {class:"body",
      header {div {class:"connection",span {class:if online{"dot online"}else{"dot"}},strong {"{conn}"} span {"{endpoint}"}} span {class:"session-state","{state}"} button {class:"button",onclick:move |_| ondisconnect.call(()),onkeydown:move |e|{if oxifoc_ui::keyboard_activate(&e){ondisconnect.call(());}}, "Disconnect"}}
      div {class:"workspace",
       main {
        div {class:"titlebar",div {h1 {match page(){0=>"Drive observatory",1=>"Controller parameters",_=>"Session journal"}} p {match page(){0=>"Currents, motion and operating conditions",1=>"Values read directly from the connected controller",_=>"Connection changes and acknowledged commands"}}}
         if page()==0 {button {class:"button",onclick:move |_| {paused.toggle();pmodel.paused.store(paused(),Ordering::Relaxed);},if paused(){"Resume plots"}else{"Pause plots"}}}
         if page()==1 {button {class:"button primary",disabled:!online,onclick:move |_|configmodel.read_config(),"Read motor parameters"}}
        }
        if !online {div {class:"notice","Waiting for the selected controller. Retained traces are stale."}}
        if page()==0 {
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
         section {class:"parameter-panel",h2 {"Motor model"} p {"Readback includes the controller revision and persistence state."} if s.config.is_empty(){p {"Select Read motor parameters to fetch the current configuration."}}else{p {class:"config-state","{s.config}"} div {class:"config-fields",for (name,value) in &s.config_fields {div {class:"config-field",span {"{name}"} strong {"{value}"}}}}}}
        } else {
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
          match current().parse::<f32>() {Ok(iq) if iq.is_finite()&&iq.abs()<=10.=>{error.set(String::new());runmodel.motor(ControlMode::CurrentControl{iq_target:iq,id_target:0.});},_=>error.set("Enter a finite current from −10 to 10 A.".into())}
         },if busy {"Waiting for acknowledgement…"} else {"Apply current"}}
         button {class:"button stop wide",disabled:!online,onclick:move |_|stopmodel.motor(ControlMode::Stopped),"Stop drive"}
         if !error().is_empty(){p {class:"error","{error}"}}
        }
        div {class:"health",h3 {"Acquisition"} div {span {"Received"} strong {"{rate} Hz"}} div {span {"Samples"} strong {"{s.samples}"}} div {span {"Active faults"} strong {"{s.faults}"}} p {if paused(){"Plots paused. Acquisition continues."}else{"Live telemetry from the controller."}}}
       }
      }
      footer {span {"OxiFOC native"} span {"{conn}"} span {"Wheel to zoom • double-click a plot to focus"}}
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
