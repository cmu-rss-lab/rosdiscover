import logging

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


def check_dead_topics(arch):
    publishers = {}  # type: Dict[Topic, Node]
    subscribers = {}  # type: Dict[Topic, Node]

    for topic in publishers:
        if topic not in subscribers:
            tpl = 'node [{}] writes to a topic without subscribers'
            for node in publishers[topic]:
                m = tpl.format(node)
                logger.error(m)

    for topic in subscribers:
        if topic not in publishers:
            tpl = 'node [{}] reads from a topic without publishers'
            for node in subscribers[topic]:
                m = tpl.format(node)
                logger.error(m)
