Add @ref xrt_view_type argument to @ref xrt_device::get_view_poses function,
this decouples the type of the view from the number. Letting us have different
semantics for the same view count.
