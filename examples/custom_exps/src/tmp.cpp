#include <bits/stdc++.h>

#include <fstream>
using namespace std;

int main(void) {
    ifstream fin("edges.txt");
    string line, token;
    int cnt = 0;
    string delimiter = " ";
    while (getline(fin, line)) {
        cout << line << endl;
        int pos = 0;
        // while ((pos = line.find(delimiter)) != string::npos) {
        //     token = line.substr(0, pos);
        //     cout << token << endl;
        //     line.erase(0, pos + delimiter.length());
        // }
        // cout << line << endl;
        // cnt++;
    }
    return 0;
}