#!/bin/bash
chown -R root:root /root
exec /bin/tini -- /usr/bin/supervisord -n
