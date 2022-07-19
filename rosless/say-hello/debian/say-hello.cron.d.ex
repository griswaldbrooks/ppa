#
# Regular cron jobs for the say-hello package
#
0 4	* * *	root	[ -x /usr/bin/say-hello_maintenance ] && /usr/bin/say-hello_maintenance
