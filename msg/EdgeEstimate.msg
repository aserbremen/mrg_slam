# Edge estimate of grah slam, in contrast to EdgeRos, the transforamtion and uncertainty is marginalized into the keyframes

uint64 gid

uint64 from_gid
uint64 to_gid

uint8 type  # 0 = odom, 1 = loop