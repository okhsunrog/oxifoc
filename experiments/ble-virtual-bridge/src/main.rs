//! Linux BLE peripheral forwarding NUS packets to oxifoc-virtual's COBS TCP.
//! Only loopback is accepted; this fixture cannot route to motor hardware.
use anyhow::{Context, Result, ensure};
use bluer::{
    Uuid,
    adv::Advertisement,
    gatt::local::{
        Application, Characteristic, CharacteristicNotify, CharacteristicNotifyMethod,
        CharacteristicWrite, CharacteristicWriteMethod, ReqError, Service,
    },
};
use futures::FutureExt;
use std::sync::Arc;
use tokio::{
    io::{AsyncBufReadExt, AsyncWriteExt, BufReader},
    sync::{Mutex, mpsc},
};
const SERVICE: Uuid = Uuid::from_u128(0x6e400001_b5a3_f393_e0a9_e50e24dcca9e);
const RX: Uuid = Uuid::from_u128(0x6e400002_b5a3_f393_e0a9_e50e24dcca9e);
const TX: Uuid = Uuid::from_u128(0x6e400003_b5a3_f393_e0a9_e50e24dcca9e);

#[tokio::main]
async fn main() -> Result<()> {
    let port: u16 = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "25267".into())
        .parse()?;
    ensure!(
        port >= 25267,
        "Use a dedicated virtual-controller port >= 25267"
    );
    let session = bluer::Session::new().await?;
    let adapter = session.default_adapter().await?;
    ensure!(adapter.is_powered().await?, "Bluetooth adapter is off");
    let (in_tx, in_rx) = mpsc::channel::<Vec<u8>>(128);
    let in_rx = Arc::new(Mutex::new(in_rx));
    let app = Application {
        services: vec![Service {
            uuid: SERVICE,
            primary: true,
            characteristics: vec![
                Characteristic {
                    uuid: RX,
                    write: Some(CharacteristicWrite {
                        write_without_response: true,
                        method: CharacteristicWriteMethod::Fun(Box::new(move |data, req| {
                            let tx = in_tx.clone();
                            async move {
                                if data.len() > 505 {
                                    return Err(ReqError::InvalidValueLength);
                                }
                                if req.offset != 0 {
                                    return Err(ReqError::InvalidOffset);
                                }
                                tx.try_send(data).map_err(|_| ReqError::Failed)
                            }
                            .boxed()
                        })),
                        ..Default::default()
                    }),
                    ..Default::default()
                },
                Characteristic {
                    uuid: TX,
                    notify: Some(CharacteristicNotify {
                        notify: true,
                        method: CharacteristicNotifyMethod::Fun(Box::new(move |mut notifier| {
                            let rx = in_rx.clone();
                            async move {
                        tokio::spawn(async move {
                            let result: Result<()> = async {
                                let mut rx = rx.try_lock().context("Only one BLE central is supported")?;
                                while rx.try_recv().is_ok() {}
                                let tcp = tokio::net::TcpStream::connect(("127.0.0.1", port)).await?;
                                tcp.set_nodelay(true)?;
                                let (read, mut write) = tcp.into_split();
                                let mut read = BufReader::new(read);
                                let mut encoded = Vec::new();
                                let mut sent = 0u64;
                                eprintln!("BLE subscription -> virtual TCP {port}");
                                loop {
                                    tokio::select! {
                                        _ = notifier.stopped() => break,
                                        data = rx.recv() => {
                                            let Some(data) = data else { break };
                                            let mut frame = cobs::encode_vec(&data); frame.push(0);
                                            write.write_all(&frame).await?;
                                        }
                                        result = read.read_until(0, &mut encoded) => {
                                            if result? == 0 { break; }
                                            ensure!(encoded.len() <= 520, "Virtual frame exceeds NUS MTU");
                                            let decoded = cobs::decode_vec(&encoded).map_err(|e| anyhow::anyhow!("COBS: {e:?}"))?;
                                            ensure!(decoded.len() <= 505, "Virtual frame exceeds NUS payload");
                                            notifier.notify(decoded).await?;
                                            encoded.clear(); sent += 1;
                                            if sent % 1000 == 0 { eprintln!("BLE notifications sent: {sent}"); }
                                        }
                                    }
                                }
                                eprintln!("BLE session ended: {sent} notifications");
                                Ok(())
                            }.await;
                            if let Err(e) = result { eprintln!("BLE bridge session: {e:#}"); }
                        });
                    }.boxed()
                        })),
                        ..Default::default()
                    }),
                    ..Default::default()
                },
            ],
            ..Default::default()
        }],
        ..Default::default()
    };
    let _gatt = adapter.serve_gatt_application(app).await?;
    let _advertising = adapter
        .advertise(Advertisement {
            service_uuids: [SERVICE].into_iter().collect(),
            local_name: Some("OxiFOC Virtual".into()),
            discoverable: Some(true),
            ..Default::default()
        })
        .await?;
    eprintln!(
        "OxiFOC Virtual NUS: {} -> 127.0.0.1:{port}",
        adapter.address().await?
    );
    tokio::signal::ctrl_c().await?;
    Ok(())
}
