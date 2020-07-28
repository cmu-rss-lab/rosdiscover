from loguru import logger

from ..interpreter import model


@model('image_transport', 'republish')
def republish(c):
    args = c.args.split(' ')
    intopic: str = 'in'
    outopic: str = 'out'
    img_format: str = 'raw'

    for a in args:
        if a in ("raw", "compressed", "theora"):
            img_format = a
        elif a.startswith("in:="):
            intopic = a[4:]
        elif a.startswith("out:="):
            outopic = a[5:]
        else:
            logger.error(f'"{a}" is not a valid argument for image_transport/republish')
    logger.info(f'The format for republish is {img_format}')
    c.sub(intopic, 'sensor_msgs/Image')
    c.pub(outopic, 'sensor_msgs/Image')
