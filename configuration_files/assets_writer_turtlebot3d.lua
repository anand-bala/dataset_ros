
VOXEL_SIZE = 5e-2

XY_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0., -math.pi / 2., 0., },
}

XZ_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0. , 0., -math.pi / 2, },
}

YZ_TRANSFORM =  {
  translation = { 0., 0., 0. },
  rotation = { 0. , 0., math.pi, },
}

options = {
  tracking_frame = "base_link",
  pipeline = {
    -- Convert intensity to color for non-grey color
    {
      action = "intensity_to_color",
      min_range = 1.,
      max_range = 60.,
    },
    {
      action = "dump_num_points",
    },
    -- Write PLY files
    {
      action = "write_ply",
      filename = "pointcloud.ply"
    },
  }
}

return options
