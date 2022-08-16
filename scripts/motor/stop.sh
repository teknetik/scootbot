kill -9 $(ps aux | grep ros | awk '{print $2}')
kill -9 $(ps aux | grep scootbot | awk '{print $2}')
