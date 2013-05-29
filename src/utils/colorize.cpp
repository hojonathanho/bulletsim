#include <vector>
#include <algorithm>
#include <sstream>

using namespace std;

int color2num (string clr) {
  static const string colors[]  = {"gray", "red", "green", "yellow", "blue", "magenta", "cyan", "white", "crimson"};
  static const vector<string> colorNames(colors, colors+9);

  int idx = find(colorNames.begin(), colorNames.end(), clr) - colorNames.begin();
  if  (idx > 8) {idx = 7;}
  return 30+idx;
}


/* Returns a colored string for display on the terminal.*/
string colorize(string text, string color, bool bold, bool highlight) {
  int num = color2num(color);
  if (highlight) num += 10;

  stringstream bh;
  bh << num;

  if (bold) {
    bh << ";";
    bh << "1";
  }

  stringstream out;
  out << "\x1b[" << bh.str() << "m" << text <<"\x1b[0m";
  return out.str();
}
