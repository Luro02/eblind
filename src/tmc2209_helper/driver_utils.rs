use esp_hal::Async;
use esp_hal::uart::{TxError, Uart};
use tmc2209::{
    ReadRequest, ReadResponse, ReadableRegister, Reader, WritableRegister, WriteRequest,
};

use super::TmcError;

/*
use crate::movement::BufferedUart;

pub async fn send_write(
    serial_address: u8,
    register: impl WritableRegister,
    writer: &mut Uart<'_, Async>,
) -> Result<(), TmcError> {
    send_write_request(WriteRequest::new(serial_address, register), writer).await
}

pub async fn send_write_request(
    write_request: WriteRequest,
    mut writer: &mut Uart<'_, Async>,
) -> Result<(), TmcError> {
    /*log::info!(
        "Sent these bytes: len({:?}) = {}",
        request.bytes(),
        request.bytes().len()
    );*/
    // Then send the request using the provided writer:

    //writer.write_all(&write_request.bytes())?;
    write_all(writer, &write_request.bytes()).await?;
    //writer.flush()?;
    // writer.write_all(&write_request.bytes()).await?;
    // writer.flush().await?;

    // await_request::<_, WriteRequest::LEN_BYTES>(writer, request.bytes()).await?;
    Ok(())
}

/*
async fn await_request<U, const N: usize>(mut reader: U, expected: &[u8]) -> Result<(), TmcError>
where
    U: embedded_io_async::Read,
{
    let mut data = [0; N];

    // TODO: create wrapper for ReadExactError
    reader.read_exact(&mut data).await.unwrap();
    if expected != &data {
        log::error!("Expected request {:?}, but got {:?}", expected, data);
    }

    Ok(())
} */

fn array_from_slice<T: Default + Clone, const N: usize>(slice: &[T]) -> [T; N] {
    core::array::from_fn(|i| {
        if i < slice.len() {
            slice[i].clone()
        } else {
            T::default()
        }
    })
}

#[esp_hal::ram]
async fn write_all(writer: &mut Uart<'_, esp_hal::Async>, mut buf: &[u8]) -> Result<(), TxError> {
    while !buf.is_empty() {
        match writer.write_async(buf).await {
            Ok(0) => panic!("write() returned Ok(0)"),
            Ok(n) => buf = &buf[n..],
            Err(e) => return Err(e),
        }
    }

    Ok(())
}

#[esp_hal::ram]
pub async fn send_and_await_read</*U, */ R>(
    serial_address: u8,
    uart: &mut Uart<'_, Async>,
) -> Result<R, TmcError>
where
    //U: embedded_io_async::Write + embedded_io_async::Read,
    R: ReadableRegister,
{
    let request = ReadRequest::new::<R>(serial_address);
    write_all(uart, request.bytes())
        .await
        .map_err(TmcError::WriteError)?;

    /*uart.write_async(request.bytes())
    .await
    .map_err(TmcError::WriteError)?;*/

    // Read the sent bytes first:
    log::info!(
        "Sent these bytes: len({:?}) = {}",
        request.bytes(),
        request.bytes().len()
    );

    let mut register_reader = Reader::default();
    let mut buffer = [0; ReadResponse::LEN_BYTES];

    loop {
        log::info!(
            "Waiting for response from driver: {:?} for {}",
            register_reader.awaiting(),
            core::any::type_name::<R>()
        );
        let read_len = uart
            .read_async(&mut buffer)
            .await
            .map_err(TmcError::ReadError)?;

        log::info!(
            "Read bytes from driver: {:?} for {}",
            &buffer[..read_len],
            core::any::type_name::<R>()
        );

        if let (_, Some(response)) = register_reader.read_response(&buffer[..read_len]) {
            if !response.crc_is_valid() {
                return Err(TmcError::InvalidCrc(array_from_slice(response.bytes())));
            }

            // Verify that the response is for the expected register:
            let addr = response.bytes()[ReadResponse::REG_ADDR_IX];
            if addr != R::ADDRESS as u8 {
                return Err(TmcError::UnexpectedAddress {
                    expected: R::ADDRESS as u8,
                    received: addr,
                });
            }

            return Ok(R::from(u32::from_be_bytes(
                response.data().try_into().unwrap(),
            )));
        }
    }
}

pub fn constrain<T: PartialOrd>(value: T, low: T, high: T) -> T {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}*/

pub trait UartExt {
    fn write_all(&mut self, buf: &[u8]) -> impl Future<Output = Result<(), TxError>>;
}

impl<'d> UartExt for Uart<'d, Async> {
    async fn write_all(&mut self, buf: &[u8]) -> Result<(), TxError> {
        let mut remaining = buf;

        while !remaining.is_empty() {
            match self.write_async(remaining).await {
                Ok(0) => panic!("write() returned Ok(0)"),
                Ok(n) => remaining = &remaining[n..],
                Err(e) => return Err(e),
            }
        }

        Ok(())
    }
}

pub async fn send_write(
    serial_address: u8,
    register: impl WritableRegister,
    writer: &mut Uart<'_, Async>,
) -> Result<(), TmcError> {
    send_write_request(WriteRequest::new(serial_address, register), writer).await
}

pub async fn send_write_request(
    write_request: WriteRequest,
    writer: &mut Uart<'_, Async>,
) -> Result<(), TmcError> {
    log::info!(
        "Sent these bytes: len({:?}) = {}",
        write_request.bytes(),
        write_request.bytes().len()
    );
    // Then send the request using the provided writer:
    writer
        .write_all(write_request.bytes())
        .await
        .map_err(TmcError::WriteError)?;
    writer.flush()?;

    await_request::<_, { WriteRequest::LEN_BYTES }>(writer, write_request.bytes()).await?;

    Ok(())
}

async fn await_request<U, const N: usize>(mut reader: U, expected: &[u8]) -> Result<(), TmcError>
where
    U: embedded_io_async::Read,
{
    let mut data = [0; N];

    // TODO: create wrapper for ReadExactError
    reader.read_exact(&mut data).await.unwrap();
    if expected != data {
        log::error!("Expected request {expected:?}, but got {data:?}");
    }

    Ok(())
}

fn array_from_slice<T: Default + Clone, const N: usize>(slice: &[T]) -> [T; N] {
    core::array::from_fn(|i| {
        if i < slice.len() {
            slice[i].clone()
        } else {
            T::default()
        }
    })
}

pub async fn send_and_await_read<R>(
    serial_address: u8,
    uart: &mut Uart<'_, Async>,
) -> Result<R, TmcError>
where
    //U: embedded_io_async::Write + embedded_io_async::Read,
    R: ReadableRegister,
{
    let request = ReadRequest::new::<R>(serial_address);
    uart.write_all(request.bytes())
        .await
        .map_err(TmcError::WriteError)?;

    // Read the sent bytes first:
    log::info!(
        "Sent these bytes: len({:?}) = {}",
        request.bytes(),
        request.bytes().len()
    );

    let mut register_reader = Reader::default();
    let mut buffer = [0; ReadResponse::LEN_BYTES];

    loop {
        log::info!(
            "Waiting for response from driver: {:?} for {}",
            register_reader.awaiting(),
            core::any::type_name::<R>()
        );
        let read_len = uart
            .read_async(&mut buffer)
            .await
            .map_err(TmcError::ReadError)?;

        log::info!(
            "Read bytes from driver: {:?} for {}",
            &buffer[..read_len],
            core::any::type_name::<R>()
        );

        if let (_, Some(response)) = register_reader.read_response(&buffer[..read_len]) {
            if !response.crc_is_valid() {
                return Err(TmcError::InvalidCrc(array_from_slice(response.bytes())));
            }

            // Verify that the response is for the expected register:
            let addr = response.bytes()[ReadResponse::REG_ADDR_IX];
            if addr != R::ADDRESS as u8 {
                return Err(TmcError::UnexpectedAddress {
                    expected: R::ADDRESS as u8,
                    received: addr,
                });
            }

            return Ok(R::from(u32::from_be_bytes(
                response.data().try_into().unwrap(),
            )));
        }
    }
}

pub fn constrain<T: PartialOrd>(value: T, low: T, high: T) -> T {
    if value < low {
        low
    } else if value > high {
        high
    } else {
        value
    }
}
