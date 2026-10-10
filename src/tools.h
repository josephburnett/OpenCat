#define PT(s) Serial.print(s)                      // abbreviate print commands
#define PT_FMT(s, format) Serial.print(s, format)  // abbreviate print commands
#define PTL(s) Serial.println(s)
#define PTF(s) Serial.print(F(s))  // trade flash memory for dynamic memory with F() function
#define PTLF(s) Serial.println(F(s))
#define PTH(head, value) \
  { \
    PT(head); \
    PT('\t'); \
    PTL(value); \
  }

//short tools
template<typename T> int8_t sign(T val) {
  return (T(0) < val) - (val < T(0));
}

void printRange(int r0 = 0, int r1 = 0) {
  if (r1 == 0)
    for (byte i = 0; i < r0; i++) {
      PT(i);
      PT('\t');
    }
  else
    for (byte i = r0; i < r1; i++) {
      PT(i);
      PT('\t');
    }
  PTL();
}

template<typename T> void printList(T *arr, byte len = DOF) {
  for (byte i = 0; i < len; i++) {  //print directly instead of building a String, to keep the heap out of the sketch
    PT(T(arr[i]));
    PTF(",\t");
  }
  PTL();
}

template<typename T> void printTable(T *list) {
  printRange(0, DOF);
  printList(list, DOF);
}

template<typename T> int strlenUntil(T *s, char terminator) {
  int l = 0;
  while (s[l++] != terminator && l < BUFF_LEN)
    ;
  return l - 1;
}
template<typename T> void printListWithoutString(T *arr, byte len = DOF) {
  for (byte i = 0; i < len; i++) {
    PT((T)(arr[i]));
    PT('\t');
  }
  PTL();
}
template<typename T> void printCmdByType(char t, T *data) {
  if (t != '\0') {
    PT(t);
    PT('\t');
    int len = (t < 'a') ? strlenUntil(data, '~') : strlen((char *)data);
    PT(len);
    PT('\t');
    if (t < 'a')
      printListWithoutString((int8_t *)data, len);
    else
      PTL((char *)data);
  }
}

//the joint angles are passed around as int8_t. the serial protocol only allows angles <= 125,
//because '~' (126) is the terminator of the binary commands. all the joint angle limits are within +-120,
//so calibratedPWM() clamps any value beyond them to the same limit as before.
int8_t clampAngle(int a) {
  return a > 125 ? 125 : (a < -125 ? -125 : (int8_t)a);
}

template<typename T, typename T1> void arrayNCPY(T *destination, const T1 *source, int len) {  //deep copy regardless of '\0'
  for (int i = 0; i < len; i++)
    destination[i] = source[i];
}

template<typename T> void getExtreme(T *arr, T *extreme, int len = DOF) {
  extreme[0] = 128;   //min
  extreme[1] = -127;  // max
  for (int i = 0; i < len; i++) {
    if (arr[i] < extreme[0])
      extreme[0] = arr[i];
    if (arr[i] > extreme[1])
      extreme[1] = arr[i];
  }
}


void resetCmd() {
  if (token == T_SKILL && newCmd[0] != '\0' && strcmp(newCmd, "rc")) {
    strcpy(lastCmd, newCmd);
  }
  newCmdIdx = 0;
  lastToken = token;
  if (token != T_SKILL && token != T_CALIBRATE && token != T_REST)
    token = '\0';
  newCmd[0] = '\0';
  cmdLen = 0;
}
