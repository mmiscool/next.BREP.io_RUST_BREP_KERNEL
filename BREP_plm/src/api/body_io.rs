//! Streaming request and response bodies on axum's own body trait, so the
//! server needs no futures combinator crate.
//!
//! * [`file_body`] streams a file out in fixed-size chunks (attachment, model
//!   and backup downloads), as `Body::from_stream` over a read loop did.
//! * [`next_chunk`] reads the next data chunk of an incoming body (uploads,
//!   bounded documents), as `into_data_stream().next()` did: trailer frames
//!   are skipped.

use axum::body::{Body, Bytes, HttpBody};
// hyper (already the TLS loop's dependency) re-exports http-body's frame type.
use hyper::body::Frame;
use std::pin::Pin;
use std::task::{Context, Poll};
use tokio::io::{AsyncRead, ReadBuf};

/// A file as a response body, read `chunk` bytes at a time; the body ends at
/// end of file and carries a read error as the body's error.
pub(crate) fn file_body(file: tokio::fs::File, chunk: usize) -> Body {
    Body::new(FileBody { file, chunk, done: false })
}

struct FileBody {
    file: tokio::fs::File,
    chunk: usize,
    done: bool,
}

impl HttpBody for FileBody {
    type Data = Bytes;
    type Error = std::io::Error;

    fn poll_frame(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Result<Frame<Bytes>, std::io::Error>>> {
        let this = &mut *self;
        if this.done {
            return Poll::Ready(None);
        }
        let mut buffer = vec![0u8; this.chunk];
        let mut read = ReadBuf::new(&mut buffer);
        match Pin::new(&mut this.file).poll_read(cx, &mut read) {
            Poll::Pending => Poll::Pending,
            Poll::Ready(Err(e)) => Poll::Ready(Some(Err(e))),
            Poll::Ready(Ok(())) => {
                let n = read.filled().len();
                if n == 0 {
                    this.done = true;
                    return Poll::Ready(None);
                }
                buffer.truncate(n);
                Poll::Ready(Some(Ok(Frame::data(Bytes::from(buffer)))))
            }
        }
    }

    fn is_end_stream(&self) -> bool {
        self.done
    }
}

/// The next data chunk of `body`, `None` at its end.
pub(crate) async fn next_chunk(body: &mut Body) -> Option<Result<Bytes, axum::Error>> {
    loop {
        let frame = std::future::poll_fn(|cx| Pin::new(&mut *body).poll_frame(cx)).await?;
        match frame {
            Err(e) => return Some(Err(e)),
            Ok(frame) => {
                if let Ok(data) = frame.into_data() {
                    return Some(Ok(data));
                }
            }
        }
    }
}
