use tmc2209::{
    ReadRequest, ReadResponse, ReadableRegister, Reader, WritableRegister, WriteRequest,
};

use super::TmcError;

pub async fn send_write(
    serial_address: u8,
    register: impl WritableRegister,
    writer: impl embedded_io_async::Write,
) -> Result<(), TmcError> {
    send_write_request(WriteRequest::new(serial_address, register), writer).await
}

pub async fn send_write_request(
    write_request: WriteRequest,
    mut writer: impl embedded_io_async::Write,
) -> Result<(), TmcError> {
    /*log::info!(
        "Sent these bytes: len({:?}) = {}",
        request.bytes(),
        request.bytes().len()
    );*/
    // Then send the request using the provided writer:
    writer.write_all(&write_request.bytes()).await?;
    writer.flush().await?;

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

pub async fn send_and_await_read<U, R>(serial_address: u8, mut uart: U) -> Result<R, TmcError>
where
    U: embedded_io_async::Write + embedded_io_async::Read,
    R: ReadableRegister,
{
    let request = ReadRequest::new::<R>(serial_address);
    uart.write_all(request.bytes()).await?;

    // Read the sent bytes first:
    log::debug!(
        "Sent these bytes: len({:?}) = {}",
        request.bytes(),
        request.bytes().len()
    );

    let mut register_reader = Reader::default();
    let mut buffer = [0; ReadResponse::LEN_BYTES];

    loop {
        log::debug!(
            "Waiting for response from driver: {:?} for {}",
            register_reader.awaiting(),
            core::any::type_name::<R>()
        );
        let read_len = uart.read(&mut buffer).await?;

        log::debug!(
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
