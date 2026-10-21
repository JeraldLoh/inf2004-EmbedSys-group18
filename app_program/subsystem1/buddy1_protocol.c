#include "buddy1_protocol.h"
static int equals(const char *p, unsigned int n, const char *s) {
 unsigned int i=0;
 while(s[i]) { if(i>=n || p[i]!=s[i]) return 0; ++i; }
 return i==n;
}
int b1_parse(const char *p, unsigned int n, b1_command *out) {
 unsigned int v=0,i;
 if(!p || !out || n>128) return 0;
 out->speed=0;
 if(equals(p,n,"START")) {out->kind=B1_START;return 1;}
 if(equals(p,n,"STOP")) {out->kind=B1_STOP;return 1;}
 if(n<11 || n>13) return 0;
 for(i=0;i<10;i++) if(p[i]!="SET_SPEED "[i]) return 0;
 for(i=10;i<n;i++) {
  if(p[i]<'0'||p[i]>'9') return 0;
  v=v*10+(unsigned int)(p[i]-'0');
 }
 if(v>100) return 0;
 out->kind=B1_SPEED;out->speed=v;return 1;
}
