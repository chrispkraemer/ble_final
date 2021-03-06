#include<stdlib.h>
#include"bst.h"
#include <stdio.h>

struct bst{
    int data;
    struct bst *left;
    struct bst *right;
};

struct bst * insert(struct bst *q,int val){
    struct bst * temp;
     if(q==NULL)
     {
     temp=(struct bst *)malloc(sizeof(struct bst));
     temp->data=val;
     temp->left=NULL;
     temp->right=NULL;
     q=temp;
     }
     else
     {
         if(val<q->data)
         {
            q->left=insert(q->left,val);            
         }
         else
         {
            q->right=insert(q->right,val);
         }
     }
     return q;  
}

void inorder(struct bst *q){
             
		 if(q==NULL)
          {                             
             return;
          }
         inorder(q->left);
         printf("%d\r\n" , q->data);
         inorder(q->right);
}

struct bst *search(struct bst *p, int key, struct bst **y){
    struct bst *temp;
     if( p == NULL)
       return(NULL);
     temp=p;
     *y = NULL;
      while( temp != NULL)
       {
         if(temp->data == key)
             return(temp);
         else
           {
             *y = temp; /*store this pointer as root */
             if(temp->data > key)
              temp = temp->left;
             else
              temp = temp->right;
           }
       }
      return(NULL);
}

/* A function to delete the node whose data value is given */
struct bst * del(struct bst *p,int val){
    struct bst *x, *y, *temp;
    x = search(p,val,&y);
    if( x==NULL)
    {
      printf("The node does not exists\r\n");
      return(p);
    }
    else
    {
/* this code is for deleting root node*/
    if(x==p)
      {
        //printf("deleting root node \r\n");
        temp = x->left;
        //printf("temp\r\n");
        //inorder(temp);
        y = x->right;
        //printf("y\r\n");
        //inorder(y);
        p = temp;
        //printf("p\r\n");
        //inorder(p);
        while(temp->right != NULL)
           temp = temp->right;
        //printf("temp\r\n");
        //inorder(temp);
        temp->right=y;
        //printf("temp\r\n");
        //inorder(temp);
        free(x);
        //printf("p\r\n");
        //inorder(p);
        return(p);
      }
/* this code is for deleting node having both children */
  if( x->left!=NULL && x->right!=NULL)
     {

       if(y->left==x)
       {
         temp=x->left;
         y->left=x->left;
         while(temp->right != NULL)
            temp = temp->right;
         temp->right=x->right;
         x->left=NULL;
         x->right=NULL;
       }
       else
        {
          temp = x->right;
          y->right = x->right;
          while(temp->left!= NULL)
             temp = temp->left;
           temp->left=x->left;
           x->left=NULL;
           x->right=NULL;
         }

       free(x);
      return(p);
    }
/* this code is for deleting a node with one child*/
   if(x->left== NULL && x->right!= NULL)
     {
       if(y->left== x)
        y->left=x->right;
        else
        y->right= x->right;
          x->right= NULL;
          free(x);
          return(p);
     }
       
	if( x->left!= NULL && x->right== NULL)
      {
           if(y->left == x)
              y->left= x->left ;
           else
              y->right= x->left;
           x->left= NULL;
           free(x);
           return(p);
      }
/* this code is for deleting a node with no child*/
    if(x->left==NULL && x->right== NULL)
       {
           if(y->left== x)
              y->left= NULL ;
           else
              y->right= NULL;
           free(x);
           return(p);
        }
    }
    printf("BAD ERROR SHOULD NOT BE HERE\r\n");
    return(p);
}