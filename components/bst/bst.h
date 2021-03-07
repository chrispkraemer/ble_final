#include<stdlib.h>
struct node;

struct node* search(struct node *root, int x);

struct node* find_minimum(struct node *root);

struct node* new_node(int x);

struct node* insert(struct node *root, int x);

struct node* delete(struct node *root, int x);

void inorder(struct node *root);