#include<stdlib.h>
struct bst;

struct bst * insert(struct bst *q,int val);

void inorder(struct bst *q);

struct bst *search(struct bst *p, int key, struct bst **y);

/* A function to delete the node whose data value is given */
struct bst * del(struct bst *p,int val);