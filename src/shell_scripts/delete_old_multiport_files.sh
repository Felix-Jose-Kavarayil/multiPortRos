# Script to delete the old data file from multiport task
# You can get it to run from the crontab 
# 0 22 * * * /home/felix/repo/multiPortRos/src/shell_scripts/delete_old_multiport_files.sh
#
find /home/felix  -maxdepth 1  -name "*.avi" -type f -mtime +7 -delete
find /home/felix  -maxdepth 1  -name "*.log" -type f -mtime +7 -delete
find /home/felix  -maxdepth 1  -name "*.protocol" -type f -mtime +7 -delete
