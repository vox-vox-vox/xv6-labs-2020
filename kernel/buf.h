struct buf {
  int valid;   // 0：invalid；1：valid     has data been read from disk?
  int disk;    // does disk "own" buf?
  uint dev;    // which device this buf belongs to?
  uint blockno;// which block-number this buf stands for?
  struct sleeplock lock;
  uint refcnt; // 引用计数？=0时说明这个buf没有被别人用。每次read这个buf时都会将这个引用计数+1，release时会将引用计数-1 。
  struct buf *prev; // LRU cache list
  struct buf *next;
  uchar data[BSIZE];
};

