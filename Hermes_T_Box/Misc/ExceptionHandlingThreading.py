import threading
import time
# Python Threading with Exception Handling
class LogThread(threading.Thread):
    """ LogThread is a class dedicated to handling
    exceptions within specific threads to prevent
    the application from crashing.
    """
    def __init__(self,msg, **kwargs):
        super().__init__(**kwargs)
        self.errmsg = msg
        self._real_run = self.run
        self.run = self._wrap_run

    def _wrap_run(self):
        try:
            self._real_run()
        except:
            print(self.errmsg)
            
def code(num):
    for i in range(num,-1,-1):
        print(2/i)
        time.sleep(1)
        
log = LogThread("Error in code!",target=code, args=(10,))
log._wrap_run()