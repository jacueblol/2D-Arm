pub mod panel;
pub mod resources;

pub use panel::{draw_panel, setup_egui_theme};
pub use resources::{PlotHistory, UiState, update_plot_history};
