#/bin/sh

. /etc/openwrt_release
VERSION_CODE=$(cat /etc/openwrt_version)

IPADDR=$(/sbin/ifconfig br-lan | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')

BASE_PORT1=19191
BASE_INTERFACE1=wlan1


BANNERTEXT=$(echo "
--------------------------------------------------------------- 
 _        _    _   _ ____           _ _       
| |      / \  | \ | |  _ \ __ _  __| (_) ___  
| |     / _ \ |  \| | |_) / _  |/ _  | |/ _ \ 
| |___ / ___ \| |\  |  _ < (_| | (_| | | (_) |
|_____/_/   \_\_| \_|_| \_\__,_|\__,_|_|\___/ 
                                              
...by CCS-Labs.org.    Version ${VERSION_CODE}          
--------------------------------------------------------------- 
                                                                
Based on ${DISTRIB_ID} ${DISTRIB_RELEASE} (lede-project.org)    
                                                                
LAN interface IPv4: ${IPADDR}                                   
Base Port ZMQ: ${BASE_PORT1}                                   
                                                                
Contact: http://www.ccs-labs.org/~klingler/                     
=============================================================== ")
