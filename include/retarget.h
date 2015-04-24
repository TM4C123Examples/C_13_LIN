#ifndef __RETARGET_H
#define	__RETARGET_H

struct __FILE { 
	void * handle;
};

typedef int (*__inputFile_t)(void);
typedef int (*__outputFile_t)(int);	

#endif
