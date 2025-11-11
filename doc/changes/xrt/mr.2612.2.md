Make it possible for the compositor to expose multiple view configurations types
supported, and letting the compositor control the sizes for them separately.
This change introduces @ref xrt_view_config and @ref xrt_view_config_properties
which are added to @ref xrt_system_compositor_info.
