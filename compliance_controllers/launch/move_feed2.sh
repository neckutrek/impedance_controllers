rosservice call /gazebo/set_link_state "link_state:
  link_name: 'link_feed2'
  pose:
    position:
      x: 3.2
      y: 0.075
      z: 0.0
    orientation: 
      x: 0.499999336576
      y: -0.500000663371
      z: 0.50000250002
      w: 0.49999750002
  twist: 
    linear: 
      x: -8.881784197e-14
      y: -5.55111512313e-15
      z: -5.55111512313e-15
    angular: 
      x: 1.87759450222e-14
      y: 3.25705428319e-14
      z: -6.66133814775e-14
  reference_frame: 'base_link'"

# rosservice call /gazebo/set_link_properties "link_name: 'link_feed2'
# com:
#   position: {x: 0.0, y: 0.0, z: 0.0}" 
