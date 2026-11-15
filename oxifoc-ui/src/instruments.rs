//! Renderer-independent instruments. None means unavailable, never zero.
use dioxus::prelude::*;

fn reading(value: Option<f32>, decimals: usize) -> String {
    value
        .filter(|v| v.is_finite())
        .map(|v| format!("{v:.decimals$}"))
        .unwrap_or_else(|| "—".into())
}
fn point(angle: f32, radius: f32) -> (f32, f32) {
    let a = (angle - 90.).to_radians();
    (160. + radius * a.cos(), 160. + radius * a.sin())
}
fn arc(from: f32, to: f32) -> String {
    let (x, y) = point(from, 124.);
    let (u, v) = point(to, 124.);
    format!(
        "M{x} {y} A124 124 0 {} 1 {u} {v}",
        u8::from(to - from > 180.)
    )
}

#[component]
pub fn SpeedDial(value: Option<f32>, #[props(default = 6000.)] limit: f32) -> Element {
    let value = value.filter(|v| v.is_finite());
    let limit = if limit.is_finite() && limit > 0. {
        limit
    } else {
        6000.
    };
    let angle = value.unwrap_or(0.).clamp(-limit, limit) / limit * 135.;
    let trace = arc(angle.min(0.), angle.max(0.));
    let (x, y) = point(angle, 124.);
    let direction = match value {
        Some(v) if v > 1. => "Forward",
        Some(v) if v < -1. => "Reverse",
        Some(_) => "Stationary",
        None => "No recent data",
    };
    rsx! { div { class: "speed-dial", role: "img", aria_label: format!("Shaft speed {} rpm. {direction}",reading(value,0)),
        svg { width:"320", height:"320", view_box:"0 0 320 320", "aria-hidden":"true",
            path { d:arc(-135.,135.), fill:"none", stroke:"#30485e", stroke_width:"8", stroke_linecap:"round" }
            for i in 0..=24 {
                {let a=-135.+i as f32*270./24.; let (x,y)=point(a,110.);let (u,v)=point(a,if i%4==0 {101.} else {106.});
                rsx!{path {d:format!("M{x} {y} L{u} {v}"),stroke:"#8299b5",stroke_width:"1.5"}}}
            }
            if value.is_some() {
                path {d:trace,fill:"none",stroke:"#6ff5e6",stroke_width:"8",stroke_linecap:"round"}
                circle {cx:"{x}",cy:"{y}",r:"5",fill:"#6ff5e6"}
            }
        }
        div {class:"dial-reading", p {"Shaft speed"} strong {class:"numeric","{reading(value,0)}"} span {"rpm"} p {class:"dial-direction","{direction}"} }
        div {class:"dial-range",span {"−{limit:.0}"} span {"0 at top"} span {"+{limit:.0}"} }
        p {class:"instrument-note","Display range ±{limit:.0} rpm"}
    } }
}

#[component]
pub fn SignedMeter(
    label: String,
    unit: String,
    value: Option<f32>,
    #[props(default = 40.)] limit: f32,
    #[props(default = "No recent data".to_string())] unavailable: String,
) -> Element {
    let value = value.filter(|v| v.is_finite());
    let limit = if limit.is_finite() && limit > 0. {
        limit
    } else {
        1.
    };
    let fraction = (value.unwrap_or(0.) / limit).clamp(-1., 1.);
    let left = 50. + fraction.min(0.) * 50.;
    let width = fraction.abs() * 50.;
    rsx! {div {class:"instrument-meter", aria_label:format!("{label}: {} {unit}",reading(value,1)),
        div {class:"instrument-heading",span {"{label}"} strong {class:"numeric","{reading(value,1)} {unit}"}}
        div {class:"meter-track",div {class:"meter-fill",style:"left:{left}%;width:{width}%"} div {class:"meter-zero"}}
        div {class:"meter-labels",span {"−{limit:.0} {unit}"} span {"0"} span {"+{limit:.0} {unit}"}}
        if value.is_none() {p {class:"instrument-note","{unavailable}"}}
        else if value.is_some_and(|v|v.abs()>limit) {p {class:"instrument-note","Outside display range"}}
    }}
}

#[component]
pub fn ThermalMeter(
    label: String,
    value: Option<f32>,
    #[props(default)] derate: Option<f32>,
    #[props(default)] trip: Option<f32>,
) -> Element {
    let value = value.filter(|v| v.is_finite());
    let derate = derate.filter(|v| v.is_finite());
    let trip = trip.filter(|v| v.is_finite());
    let scale = trip.unwrap_or(120.).max(120.) * 1.15;
    let width = value.unwrap_or(0.).clamp(0., scale) / scale * 100.;
    let danger = value.zip(trip).is_some_and(|(v, t)| v >= t);
    rsx! {div {class:if danger {"thermal-meter danger"} else {"thermal-meter"},
        div {class:"instrument-heading",span {"{label}"} strong {class:"numeric","{reading(value,1)} °C"}}
        div {class:"meter-track",div {class:"meter-fill thermal-fill",style:"width:{width}%"}
            if let Some(t)=derate {div {class:"threshold",style:format!("left:{}%",(t/scale*100.).clamp(0.,100.)),title:format!("Derating {t} °C")}}
            if let Some(t)=trip {div {class:"threshold trip",style:format!("left:{}%",(t/scale*100.).clamp(0.,100.)),title:format!("Trip {t} °C")}}
        }
        p {class:"instrument-note",if let Some((v,t))=value.zip(derate) {"{(t-v).max(0.):.1} °C to derating"} else {"Derating limit unavailable"}}
        p {class:"instrument-note",if let Some(t)=trip {"Trip {t:.0} °C"} else {"Trip limit unavailable"}}
    }}
}

#[component]
pub fn Instruments(
    rpm: Option<f32>,
    iq: Option<f32>,
    fet: Option<f32>,
    motor: Option<f32>,
) -> Element {
    rsx! {section {class:"instruments", aria_label:"Drive instruments",
        SpeedDial {value:rpm}
        div {class:"instrument-stack",
            SignedMeter {label:"Torque current · Iq",unit:"A",value:iq}
            SignedMeter {label:"DC bus power",unit:"W",value:None,limit:2000.,unavailable:"Not provided by telemetry"}
            div {class:"thermal-pair",ThermalMeter {label:"MOSFET",value:fet} ThermalMeter {label:"Motor",value:motor}}
            p {class:"instrument-note","Fixed display ranges; controller limits are separate."}
        }
    }}
}
