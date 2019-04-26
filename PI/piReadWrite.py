#receives and sends data to the arduino
#manages localstate for pi
import serial
import time
import State
import json, os

from State import State

#Initialize State
root = State()

#initialize Serial Information
try:
    ser = serial.Serial('/dev/ttyACM0',9600) #ls /dev/tty* replace with correct ACM connection
except:
    ser = serial.Serial('/dev/ttyACM1',9600)
#ser = serial.Serial('COM4',9600)
ser.baudrate=9600

def sendBlockData(root):
    sendData(root.size)
    for x in root.blockX:
        sendData(x)
        
    for y in root.blockY:
        sendData(y)

def sendData(val):
    msg = str(val)
    print('sending:', msg)
    ser.write(msg.encode('utf-8'))
    time.sleep(1)
    
def receiveData(root):
    print('listening...')
    read_serial=ser.readline()
    if(read_serial):
        print('received', read_serial.decode('utf-8')) #decode data and print
        root.val = 'pi' #set pi as active
    time.sleep(1)

'''Parses json file and returns coordinate values as a string as a whole.'''
def parsJson(root):    
    '''sets ylocs, xlocs, and size to values given by json'''

    
    #path = os.getcwd() + '\\resources\\' + 'file.json' # specifing relative path. #windows
    path = os.getcwd() + '/resources/file.json' #linux

    fileObj = open(path)                #Creating a fileObject                       
    fileReadObj = fileObj.read()        #Creating a fileReadObject
    py_values = json.loads(fileReadObj) #py_values are dictionary values.


    #typecasting dictionary py_values to tuple_values
    tuple_values = tuple(py_values.values())
    #storing x values and y values in coordinates
    root.blockX = py_values.get("x coords")
    root.blockY = py_values.get("y coords")
    root.size = int(py_values.get("size"))
    
    fileObj.close()

if __name__ == '__main__':
    while True:
        if(root.val == 'pi'):
            if(root.blockDataSent == False):
                parsJson(root)
                sendBlockData(root)
                root.val = "done"
            else:
                pass
        elif(root.val == 'ard'):
            receiveData(root)
