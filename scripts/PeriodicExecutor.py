import sched
import time
import threading
import signal

class PeriodicExecutor:
    def __init__(self, frequency):
        self.interval = 1 / frequency
        self.scheduler = sched.scheduler(time.time, time.sleep)
        self.thread = threading.Thread(target=self.scheduler.run)
        self.thread.daemon = True
        self.next_time = None
        self.stop_event = threading.Event()

    def periodic_function(self):
        # This method should be overridden in the derived class
        raise NotImplementedError("This method should be overridden by subclasses")

    def schedule_periodic_function(self):
        if self.next_time is not None:
            current_time = time.time()
            if current_time > self.next_time + self.interval:
                print("Missed deadline at", current_time)
        
        if not self.stop_event.is_set():
            self.next_time = time.time() + self.interval
            self.scheduler.enterabs(self.next_time, 1, self.schedule_periodic_function)
            self.periodic_function()

    def start(self):
        self.scheduler.enterabs(self.interval, 1, self.schedule_periodic_function)
        self.thread.start()

    def join(self):
        self.thread.join()

    def stop(self):
        self.stop_event.set()

if __name__ == "__main__":
    class MyExecutor(PeriodicExecutor):
        def __init__(self, interval):
            super().__init__(interval)

        def periodic_function(self):
            print(f"Function executed at {time.time()}")

    executor = MyExecutor(1000)

    def signal_handler(signum, frame):
        print("Signal handler called with signal", signum)
        executor.stop()

    signal.signal(signal.SIGINT, signal_handler)
    
    executor.start()

    try:
        executor.join()
    except KeyboardInterrupt:
        print("Stopped by user")
