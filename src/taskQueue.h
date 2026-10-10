// A queue of pending tasks, used by the modules (e.g. ultrasonic.h) to schedule a
// short sequence of commands with delays in between.
//
// It used to be a QList of heap-allocated Task objects (new Task / new char[] / delete).
// It is now a fixed-capacity ring buffer of statically allocated tasks, so that
// malloc/realloc/free are not linked into the sketch (~900 bytes of flash) and the
// RAM usage is known at compile time.

#define TASK_QUEUE_CAPACITY 4  //the longest sequence enqueued at once is 3 (readRGBultrasonic())
#define TASK_PARAMETER_LEN 20  //the longest parameter list is the 16-byte 'L' frame + the '~' terminator

long taskTimer = 0;
long taskInterval = -1;

struct Task {
  char tkn;
  byte paraLength;
  int dly;
  char parameters[TASK_PARAMETER_LEN];
};

class TaskQueue {
  Task tasks[TASK_QUEUE_CAPACITY];
  byte head = 0;   //index of the oldest queued task
  byte count = 0;  //number of queued tasks
public:
  byte size() {
    return count;
  }
  template<typename T> void addTask(char t, T *p, int d = 0) {
    if (count >= TASK_QUEUE_CAPACITY)  //the queue is full: the new task is dropped
      return;
    Task *task = &tasks[(head + count) % TASK_QUEUE_CAPACITY];
    count++;
    task->tkn = t;
    task->dly = d;
    int len = (t >= 'A' && t <= 'Z') ? strlenUntil(p, '~') : strlen((char *)p);
    if (len > TASK_PARAMETER_LEN - 1)  //truncate rather than overflow the buffer
      len = TASK_PARAMETER_LEN - 1;
    task->paraLength = len;
    arrayNCPY(task->parameters, p, len);
    task->parameters[len] = (token >= 'A' && token <= 'Z') ? '~' : '\0';  //as in the original Task(), the terminator \
                                                                         //follows the current token. it's never read back, \
                                                                         //because loadTaskInfo() copies only paraLength bytes.
  }
  bool empty() {
    return count == 0 && long(millis() - taskTimer) > taskInterval;
  }
  void loadTaskInfo(Task *t) {
    token = t->tkn;
    cmdLen = t->paraLength;
    taskInterval = t->dly;
    arrayNCPY(newCmd, t->parameters, cmdLen);
    newCmd[cmdLen] = (token >= 'A' && token <= 'Z') ? '~' : '\0';
    taskTimer = millis();
    newCmdIdx = 5;
  }
  void popTask() {
    if (count > 0 && long(millis() - taskTimer) > taskInterval) {
      loadTaskInfo(&tasks[head]);
      head = (head + 1) % TASK_QUEUE_CAPACITY;
      count--;
    }
  }
};
static TaskQueue taskQueue;
static TaskQueue *const tQueue = &taskQueue;
