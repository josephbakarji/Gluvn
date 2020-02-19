from threading import Thread
import mido
import time

class read_keyboard(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.portname = 'Digital Piano'
        #self.dataq = dataq
        self.daemon = True

    def run(self):
        inport = mido.open_input(self.portname)
        print('Piano Keyboard Port Name: ', self.portname)
        while True:
            msg = inport.receive(block=True)
            #self.dataq.put(msg)
            print(msg)


class print_in_parallel(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True

    def run(self):
        d = 0
        while True:
            d = d + 1
            time.sleep(0.2)
            print('whatever', d)

key_thread = read_keyboard()
printtread = print_in_parallel()

key_thread.start()
printtread.start()

while True:
    key_thread.join()
    printtread.join()