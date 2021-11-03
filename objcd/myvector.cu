//
// Created by zhuoc on 2021/11/2.
//

#include "myvector.h"

__host__ __device__
LinkList::LinkList(int size) {
    array = new int[size];
}

__host__ __device__
LinkList::~LinkList() {
    delete array;
}

__host__ __device__
MyVector::~MyVector() {
    auto p = head;
    while (p) {
        auto next = p->next;
        delete p;
        p = next;
    }
}

__host__ __device__
int MyVector::ui(int idx) {
    return idx / sizePerArray;
}

__host__ __device__
int MyVector::li(int idx) {
    return idx % sizePerArray;
}

__host__ __device__
void MyVector::add_array() {
    if (!head) {
        head = tail = new LinkList(sizePerArray);
    } else {
        tail->next = new LinkList(sizePerArray);
        tail = tail->next;
    }
    nArray++;
    mCapacity += sizePerArray;
}

__host__ __device__
int *MyVector::get_array(int idx) {
    auto p = head;
    for (int i = 0; i < idx; i++) {
        p = head->next;
    }
    return p->array;
}

__host__ __device__
void MyVector::push_back(int v) {
    if (mSize + 1 > mCapacity) {
        add_array();
    }
    tail->array[li(mSize)] = v;
    mSize ++;
}

__host__ __device__
int MyVector::size() const {
    return mSize;
}

__host__ __device__
int MyVector::operator[] (int idx) {
    idx %= mSize;
    if (idx < 0)
        idx = mSize + idx;
    return get_array(ui(idx))[li(idx)];
}
