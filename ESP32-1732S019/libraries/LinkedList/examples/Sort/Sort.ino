/*
	LinkedList Example
	Link: http://github.com/PaulMurrayCbr/LinkedList
	Forked from: http://github.com/ivanseidel/LinkedList

	Example Created by
		Paul Murray, github.com/PaulMurrayCbr
*/

#include <LinkedList.h>

char testString[] = "Lorem ipsum dolor sit amet, \
consectetur adipiscing elit, sed do eiusmod tempor \
incididunt ut labore et dolore magna aliqua. Ut enim \
ad minim veniam, quis nostrud exercitation ullamco \
laboris nisi ut aliquip ex ea commodo consequat. Duis \
aute irure dolor in reprehenderit in voluptate velit \
esse cillum dolore eu fugiat nulla pariatur. Excepteur \
sint occaecat cupidatat non proident, sunt in culpa qui \
officia deserunt mollit anim id est laborum.";

LinkedList<char *> list;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.print("Beginning sketch in ");
  for (int i = 3; i > 0; i--) {
    Serial.print(i);
    Serial.print(' ');
    delay(500);
  }
  Serial.println("0.");

  Serial.println("Loading some strings into the linked list");
  Serial.println();

  char *p;
  int col = 0;
  for (p = strtok(testString, " ,."); p; p = strtok(NULL, " ,.")) {
    Serial.print(p);
    Serial.print(' ');
    col += strlen(p) + 1;
    if (col >= 80) {
      Serial.println();
      col = 0;
    }
    list.add(p);
  }


  if (col) {
    Serial.println();
    col = 0;
  }
  
  Serial.println();
  Serial.println("Sorting");
  Serial.println();

  list.sort(compare);

  Serial.println();
  Serial.println("Result");
  Serial.println();

  while (p = list.shift()) {
    Serial.print(p);
    Serial.print(' ');
    col += strlen(p) + 1;
    if (col >= 80) {
      Serial.println();
      col = 0;
    }
  }
  if (col) {
    Serial.println();
    col = 0;
  }

  Serial.println();
  Serial.println("Done!");
}

int compare(char *&a, char *&b) {
  return strcmp(a, b);
}

void loop() {
  // put your main code here, to run repeatedly:

}
