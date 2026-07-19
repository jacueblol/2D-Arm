use std::time::SystemTime;

use colored::{Color, Colorize};

const INFO: &str = "[INFO]";
const OK: &str = "[OK]";
const WARN: &str = "[WARN]";
const ERROR: &str = "[ERROR]";
const DEBUG: &str = "[DEBUG]";

/// Finds the current timestamp in [u128] and then converts it to hours, min, secs, and millis
fn timestamp() -> String {
    let now = SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis();

    // Convert to readable HH:MM:SS.mmm
    let millis = now % 1000;
    let secs = (now / 1000) % 60;
    let mins = (now / (1000 * 60)) % 60;
    let hours = (now / (1000 * 60 * 60)) % 24;

    format!("{:02}:{:02}:{:02}.{:03}", hours, mins, secs, millis)
}

/// Gets the color that is assigned to each loglevel
fn print_c_helper(variable: & impl AsRef<str>) -> Color {
    return match variable.as_ref() {
        INFO => Color::Blue,
        OK => Color::Green,
        WARN => Color::Yellow,
        ERROR => Color::Red,
        DEBUG => Color::Magenta,
        _ => "".to_string().into(),
    }

}

/// prints the corrisponding msg in the correct color with a timestamp at the log level assigned
fn print_c(category: impl AsRef<str>, subsystem: impl AsRef<str>, msg: impl AsRef<str>) {

    let color = print_c_helper(&category);
    
    let _category = category.as_ref().color(color);
    let _subsystem = subsystem.as_ref().color(color);
    let _timestamp = timestamp().dimmed().color(color);
    print!("{} ", _timestamp);
    
    print!("{} ", _category);

    if !subsystem.as_ref().is_empty() {
        print!("{} ", _subsystem);
    } else {
        print!("")
    }

    println!("{}", msg.as_ref().to_string());
}

/// info log using [print_c]
pub fn info(subsystem: impl AsRef<str>, msg: impl AsRef<str>) {
    print_c(INFO, subsystem, msg);
}

/// sucuccess log using [print_c]
pub fn success(subsystem: impl AsRef<str>, msg: impl AsRef<str>) {
    print_c(OK, subsystem, msg);
}

#[allow(dead_code)]
pub fn warn(subsystem: impl AsRef<str>, msg: impl AsRef<str>) {
    print_c(WARN, subsystem, msg);
}

#[allow(dead_code)]
pub fn error(subsystem: impl AsRef<str>, msg: impl AsRef<str>) {
    print_c(ERROR, subsystem, msg);
}

#[allow(dead_code)]
pub fn debug(subsystem: impl AsRef<str>, msg: impl AsRef<str>) {
    print_c(DEBUG, subsystem, msg);
}