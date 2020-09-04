import time
import socket
import queue

from log     import *
from globals import *

class DataReporter:

    def __init__(self, port):

        self.address    = None
        self.port       = port
        self.data_queue = queue.Queue()

    def connect(self, address):

        self.address = address

    def enqueue_data(self, data):

        self.data_queue.put(data)

    def __dequeue_data__(self):

        if self.data_queue.empty():
            return None
        else:
            return self.data_queue.get()

    def __is_data_available__(self):

        return not self.data_queue.empty()

    def start(self):

        while True:

            if self.address is None:

                time.sleep(IDLE_LOOP_SLEEP_TIME)

            else:

                log(INFO, 'Initiating data reporter')

                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:

                    try:
                        client_socket.connect((self.address, self.port))
                    except Exception as e:
                        log(WARNING, 'Data reporter connection error')
                        log(WARNING, e                               )
                        client_socket.close()
                        time.sleep(1)
                        continue

                    log(DEBUG, 'Connected to '+ str(self.address) + "/" + str(self.port))

                    while True:

                        if self.__is_data_available__():

                            out_data  = self.__dequeue_data__()
                            out_data += "\n"

                            try:
                                client_socket.sendall(out_data.encode())
                            except Exception as e:
                                log(WARNING, 'Server disconnected (with error)')
                                log(WARNING, e                                 )
                                client_socket.close()
                                break

                        else:

                            time.sleep(IDLE_LOOP_SLEEP_TIME)
