#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "date.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"

uint64
sys_exit(void)
{
  int n;
  if(argint(0, &n) < 0)
    return -1;
  exit(n);
  return 0;  // not reached
}

uint64
sys_getpid(void)
{
  return myproc()->pid;
}

uint64
sys_fork(void)
{
  return fork();
}

uint64
sys_wait(void)
{
  uint64 p;
  if(argaddr(0, &p) < 0)
    return -1;
  return wait(p);
}

uint64
sys_sbrk(void)
{
  int addr;
  int n;

  if(argint(0, &n) < 0)
    return -1;
  addr = myproc()->sz;
  if(growproc(n) < 0)
    return -1;
  return addr;
}

uint64
sys_sleep(void)
{
  int n;
  uint ticks0;
  backtrace();
  if(argint(0, &n) < 0)
    return -1;
  acquire(&tickslock);
  ticks0 = ticks;
  while(ticks - ticks0 < n){
    if(myproc()->killed){
      release(&tickslock);
      return -1;
    }
    sleep(&ticks, &tickslock);
  }
  release(&tickslock);
  return 0;
}

uint64
sys_kill(void)
{
  int pid;

  if(argint(0, &pid) < 0)
    return -1;
  return kill(pid);
}

// return how many clock tick interrupts have occurred
// since start.
uint64
sys_uptime(void)
{
  uint xticks;

  acquire(&tickslock);
  xticks = ticks;
  release(&tickslock);
  return xticks;
}

uint64
sys_sigalarm(void)
{
  struct proc* p = myproc();
  int ticks_required;
  uint64 handler_ptr;
  // argint(a,b) a是第几个参数，b是对应接受指针，其他函数同理。
  if(argint(0,&ticks_required) < 0 || argaddr(1, &handler_ptr) < 0){
    return -1;
  }
  // if(ticks_required==0) return 0;// 如果传入0，则不进行处理
  p->ticks_required=ticks_required;// 设置触发handler需要的cpu周期数
  p->handler_ptr=handler_ptr;//设置handler
  return 0;
}

uint64
sys_sigreturn(void)
{
  struct proc* p = myproc();
  *(p->trapframe)=p->trapframe_cp;
  p->ticks_now=0;
  p->inhandler=0;//取消inhandler标志，允许响应接下来的interrupt
  return 0;
}