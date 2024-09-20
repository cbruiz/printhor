use futures::stream;
use futures_executor::block_on;
use std::io::Read;

#[derive(Debug)]
enum Error {
    Io(std::io::Error),
    Parse(async_gcode::Error),
}
impl From<async_gcode::Error> for Error {
    fn from(f: async_gcode::Error) -> Self {
        Self::Parse(f)
    }
}
fn main() {
    block_on(async {
        let mut parser = async_gcode::Parser::new(stream::iter(
            std::io::stdin().bytes().map(|res| res.map_err(Error::Io)),
        ));

        while let Some(res) = parser.next().await {
            println!("{:?}", res);
        }
    });
}
