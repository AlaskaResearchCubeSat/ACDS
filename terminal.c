#include <msp430.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "UCA1_uart.h"
#include "terminal.h"

//make printf send over USB
int __putchar(int ch){
  return UCA1_TxChar(ch);
}

//used to break up a string into arguments for parsing
//*argv[] is a vector for the arguments and contains pointers to each argument
//*dst is a buffer that needs to be big enough to hold all the arguments
//returns the argument count
unsigned short make_args(char *argv[],const char *src,char *dst){
    unsigned short argc=0;
    argv[0]=dst;
    for(;;){
        while(isspace(*src))src++;
        //copy non space characters to dst
        while(!isspace(*src) && *src)
            *dst++=*src++;
        //terminate string bit
        *dst++=0;
        //at the end of src?
        if(*src==0)break;
        argc++;
        argv[argc]=dst;
    }
    //don't count null strings
    if(*argv[argc]==0)argc--;
    return argc;
}

//print a list of all commands
int helpCmd(char **argv,unsigned short argc){
  int i,rt=0;
  if(argc!=0){
    //arguments given, print help for given command
    //loop through all commands
    for(i=0;cmd_tbl[i].name!=NULL;i++){
      //look for a match
      if(!strcmp(cmd_tbl[i].name,argv[1])){
        //match found, print help and exit
        printf("%s %s\r\n",cmd_tbl[i].name,cmd_tbl[i].helpStr);
        return 0;
      }
    }
    //no match found print error
    printf("Error : command \'%s\' not found\r\n",argv[1]);
    //fall through and print a list of commands and return -1 for error
    rt=-1;
  }
  //print a list of commands
  printf("Possible Commands:\r\n");
  for(i=0;cmd_tbl[i].name!=NULL;i++){
    printf("\t%s\r\n",cmd_tbl[i].name);
  }
  return rt;
}

//execute a command
int doCmd(char *cs){
  //buffers for args and arg vector
  //NOTE: this limits the maximum # of arguments
  //      and total length of all arguments
  char args[50];
  char *argv[10];
  unsigned short argc;
  int i;
  //split string into arguments
  argc=make_args(argv,cs,args);
  //search for command
  for(i=0;cmd_tbl[i].name!=NULL;i++){
    //look for a match
    if(!strcmp(cmd_tbl[i].name,argv[0])){
      //match found, run command and return
      return cmd_tbl[i].cmd(argv,argc);
    }
  }
  //unknown command, print help message
  printf("unknown command \'%s\'\r\n",argv[0]);
  helpCmd(NULL,0);
  //unknown command, return error
  return 1;
}

//task to communicate with the user over USB
void terminal(void *p) __toplevel{
  //character from port
  int c=0;
  //buffer for command
  char cmd[50];
  //buffer for last command
  char last[50];
  //command string index
  unsigned int cIdx=0;
  cmd[0]=0;
  last[0]=0;
  //write prompt
  putchar('>');
  for(;;){
    //get character
    c=UCA1_Getc();
    //process received character
    switch(c){
      case '\r':
      case '\n':
        //return key run command
        if(cIdx==0){
        //if nothing entered, do last command
        printf(last);       //print command
        strcpy(cmd,last);   //copy into command buffer
        }else{
        //run command from buffer
        cmd[cIdx]=0;    //terminate command string
        cIdx=0;         //reset the command index
        //save this as last command
        strcpy(last,cmd);
        }
        //send carriage return and new line
        printf("\r\n");
        //run command
        doCmd(cmd);
        //print prompt char
        putchar('>');
        continue;
      case '\x7f':
      case '\b':
        //backspace
        if(cIdx==0)continue;
        //backup and write over char
        printf("\b \b");
        //decrement command index
        cIdx--;
        continue;
      case '\t':
        //ignore tab character
        continue;
    }
    //check for control char
    if(!iscntrl(c) && cIdx<(sizeof(cmd)/sizeof(cmd[0]) - 1)){
      //echo character
      putchar(c);
      //put character in command buffer
      cmd[cIdx++]=c;
    }
  } 
}

