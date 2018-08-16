import requests
import json

import paho.mqtt.client as mqttClient
import time
 
def on_connect(client, userdata, flags, rc):
 
    if rc == 0:
 
        print("Connected to broker")
 
        global Connected                #Use global variable
        Connected = True                #Signal connection 
 
    else:
 
        print("Connection failed")
 
def on_message(client, userdata, message):
    
    print "Message received: "  + message.payload
    print "................................                     ."
    url = 'https://smartcity.rbccps.org/api/0.1.0/publish'
    #payload = {'exchange' : 'amq.topic', 'key' : '5CCF7F3D77782', 'body' : '{ "key" : "5CCF2F3D7782" , "data" :"'+str(message.payload)+'"}' }
    payload = str(message.payload)
    print(payload)
    headers = {'apikey': '453477c5165341f9a3264637e66519b9'}
    r = requests.post(url, verify=False,  data=payload, headers=headers )
    print(payload)
    print r.status_code
    print r.json

#    print (message.payload)
 
Connected = False   #global variable for the state of the connection
 
broker_address= "172.24.2.2"  #Broker address
port = 1883                         #Broker port
user = "Raspberry_Pi_1"                    #Connection username
password = "Raspberry1"            #Connection password
 
client = mqttClient.Client("Raspberry_Pi")               #create new instance
client.username_pw_set(user, password=password)    #set username and password
client.on_connect= on_connect                      #attach function to callback
client.on_message= on_message                      #attach function to callback
 
client.connect(broker_address, port=port)          #connect to broker
 
client.loop_start()        #start the loop
while Connected != True:    #Wait for connection
    time.sleep(0.1)
 
#client.subscribe("ESP4/test_ESP4")
client.subscribe("ESP4/test_ESP4")
client.subscribe("ESP3/test_ESP3") 
client.subscribe("ESP2/test_ESP2")
try:
    while True:
        time.sleep(1)
 
except KeyboardInterrupt:
    print "exiting"
    client.disconnect()
    client.loop_stop()



