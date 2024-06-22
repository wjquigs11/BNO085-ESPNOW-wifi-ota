// compass control structure
typedef struct control_s {
  bool compassOnToggle = true;
  int orientation = 0;
  int variation = 0;
  int frequency = 100;
  char hostname[64] = "";
} control_s;
extern control_s compassParams;
