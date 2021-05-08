
#!/bin/bash

#curr=$(awk '{x="'"`date +%Y%M%d%S%N`"'"; print x}')
#echo $curr
start_time=$((10#$(date +'%s') * 1000 + 10#$(date +'%N') / 1000000))
echo -e 'time\tframes'

while true; do
    curr_time=$((10#$(date +'%s') * 1000 + 10#$(date +'%N') / 1000000))
    rel_time=$(( 10#$curr_time - 10#$start_time))
    #rel_time="27"
    #sudo cat /sys/kernel/debug/ieee80211/phy1/ath9k/queues | awk '{x="'"`date +%Y%M%d%S%N`"'"; if ($1 == "(BE):") print x"\t"$9;}'
    cat /sys/kernel/debug/ieee80211/phy1/ath9k/queues | awk -v rel_time=$rel_time '{if ($1 == "(BE):") print rel_time"\t"$9;}'
    sleep 0.08001s
done
