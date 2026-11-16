"""Display-only geometry identity. Source poses/timestamps are never changed."""
def path_visual_key(points):
    # One centimetre is finer than the 5 cm navigation voxel and the normal
    # map pixel. DDS heartbeat stamps must not invalidate raster geometry.
    return tuple((round(float(x),2),round(float(y),2))for x,y in points)


def model_visual_key(model,joints,tilt):
    return (id(model),tuple(sorted((name,round(float(value),3))for name,value in joints.items())),
            tuple(round(float(value),3)for value in tilt))
