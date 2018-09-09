#include <stdio.h>
#include <stdlib.h>

void main(void){
	printf("starting test\n");
	int pos=0;
	while(1){
		printf("start\n");
		pos=(++pos)%4;
		printf("pos: %x;  %x %x %x %x\n",pos, pos==1, 1 & pos, 0, 0, 0);
		sleep(1);
	}
}
