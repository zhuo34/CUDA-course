//
// Created by zhuoc on 2021/11/1.
//

#include "bvh.h"
#include "myobj.h"

#include <iostream>

BVHNode *BVHNode::build(BVHNode **nodes, int n, int depth, int idx) {
    if (n == 1) {
        nodes[0]->depth = depth;
        nodes[0]->idx = idx;
        return nodes[0];
    }
    vec3f ctrMin = nodes[0]->ctr;
    vec3f ctrMax = nodes[0]->ctr;
    for (int i = 1; i < n; i++) {
        ctrMin.setMin(nodes[i]->ctr);
        ctrMax.setMax(nodes[i]->ctr);
    }
    int axis = 0;
    float maxSpan = ctrMax[0] - ctrMin[0];
    for (int i = 1; i < 3; i++) {
        float span = ctrMax[i] - ctrMin[i];
        if (span > maxSpan) {
            maxSpan = span;
            axis = i;
        }
    }

    if (axis == 0)
        qsort(nodes, n, sizeof(BVHNode*), box_x_cmp);
    else if(axis == 1)
        qsort(nodes, n, sizeof(BVHNode*), box_y_cmp);
    else
        qsort(nodes, n, sizeof(BVHNode*), box_z_cmp);

    auto root = new BVHNode;
    root->l = build(nodes, n / 2, depth + 1, idx);
    root->r = build(nodes + n / 2, n - n/2, depth + 1, root->l->idx + 1);

    vec3f a(myfmin(root->l->a.x, root->r->a.x), myfmin(root->l->a.y, root->r->a.y), myfmin(root->l->a.z, root->r->a.z));
    vec3f b(myfmax(root->l->b.x, root->r->b.x), myfmax(root->l->b.y, root->r->b.y), myfmax(root->l->b.z, root->r->b.z));
    root->setBox(a, b);
    root->height = root->l->height + 1;
    root->depth = depth;
    root->idx = root->r->idx + 1;
    root->nodeCnt = 1 + root->l->nodeCnt + root->r->nodeCnt;
    return root;
}

int BVHNode::box_x_cmp(const void *a, const void *b) {
    auto *ah = *(BVHNode**)a;
    auto *bh = *(BVHNode**)b;

    if (ah->ctr.x < bh->ctr.x)
        return -1;
    else
        return 1;
}

int BVHNode::box_y_cmp(const void *a, const void *b) {
    auto *ah = *(BVHNode**)a;
    auto *bh = *(BVHNode**)b;

    if (ah->ctr.y < bh->ctr.y)
        return -1;
    else
        return 1;
}

int BVHNode::box_z_cmp(const void *a, const void *b) {
    auto *ah = *(BVHNode**)a;
    auto *bh = *(BVHNode**)b;

    if (ah->ctr.z < bh->ctr.z)
        return -1;
    else
        return 1;
}

int BVHNode::box_idx_cmp(const void *a, const void *b) {
    auto *ah = *(BVHNode**)a;
    auto *bh = *(BVHNode**)b;

    if (ah->triIdx < bh->triIdx)
        return -1;
    else
        return 1;
}

int BVHNode::contact(BVHNode *node, const MyObj *obj, std::set<std::pair<int, int>> &pairs) {
//    if (!isLeaf())
//        return;
    int cnt = 0;
    if (node->isLeaf() && node->triIdx == triIdx)
        return cnt;
    if (contact(node)) {
        if (node->isLeaf()) {
            if (obj->triContactDetection(triIdx, node->triIdx)) {
                cnt++;
                int i = triIdx < node->triIdx ? triIdx : node->triIdx;
                int j = triIdx + node->triIdx - i;
                pairs.emplace(i, j);
            }
        } else {
            cnt += contact(node->l, obj, pairs);
            cnt += contact(node->r, obj, pairs);
        }
    }
    return cnt;
}

bool BVHNode::contact(BVHNode *node) const {
    bool cond_x = (a.x - node->b.x) * (b.x - node->a.x) <= 0;
    bool cond_y = (a.y - node->b.y) * (b.y - node->a.y) <= 0;
    bool cond_z = (a.z - node->b.z) * (b.z - node->a.z) <= 0;
    return cond_x && cond_y && cond_z;
}

int BVHNode::count() {
    return nodeCnt;
}

BVHDenseNode *BVHNode::dense() {
    auto *bvh = new BVHDenseNode[count()];
    _dense(bvh, count());
    return bvh;
}

void BVHNode::_dense(BVHDenseNode *bvh, int cnt) {
    if (!isDensed) {
        idx = cnt - 1 - idx;
        isDensed = true;
    }
    bvh[idx].triIdx = triIdx;
    bvh[idx].a = a;
    bvh[idx].b = b;
    if (isLeaf())
        return;
    l->_dense(bvh, cnt);
    r->_dense(bvh, cnt);
    bvh[idx].l = l->idx;
    bvh[idx].r = r->idx;
}

__host__ __device__
bool boxContact(const vec3f &a1, const vec3f &b1, const vec3f &a2, const vec3f &b2) {
    bool cond_x = (a1.x - b2.x) * (b1.x - a2.x) <= 0;
    if (!cond_x)
        return false;
    bool cond_y = (a1.y - b2.y) * (b1.y - a2.y) <= 0;
    if (!cond_y)
        return false;
    bool cond_z = (a1.z - b2.z) * (b1.z - a2.z) <= 0;
    if (!cond_z)
        return false;
    return true;
}

__host__ __device__
bool BVHDenseNode::contact(BVHDenseNode *node) const {
    bool cond_x = (a.x - node->b.x) * (b.x - node->a.x) <= 0;
    if (!cond_x)
        return false;
    bool cond_y = (a.y - node->b.y) * (b.y - node->a.y) <= 0;
    if (!cond_y)
        return false;
    bool cond_z = (a.z - node->b.z) * (b.z - node->a.z) <= 0;
    return cond_z;
}

int BVHDenseNode::contact(BVHDenseNode *bvh, int idx, const MyObj *obj, std::set<std::pair<int, int>> &pairs) {
//    if (!isLeaf())
//        return;
    int cnt = 0;
    auto node = bvh + idx;
    if (node->isLeaf() && node->triIdx == triIdx)
        return cnt;
    if (contact(node)) {
        if (node->isLeaf()) {
            if (obj->triContactDetection(triIdx, node->triIdx)) {
                cnt++;
                int i = triIdx < node->triIdx ? triIdx : node->triIdx;
                int j = triIdx + node->triIdx - i;
                pairs.emplace(i, j);
            }
        } else {
            cnt += contact(bvh, node->l, obj, pairs);
            cnt += contact(bvh, node->r, obj, pairs);
        }
    }
    return cnt;
}

__host__ __device__
int BVHDenseNode::contact(BVHDenseNode *bvh, int idx, vec3f *d_vs, Triangle *d_fs, MyVector *d_res) {
    auto *node = &bvh[idx];
    if (node->isLeaf() && node->triIdx == triIdx)
        return 0;
    if (contact(node)) {
        if (node->isLeaf()) {
            if (MyObj::triContactDetectionCUDA(d_vs, d_fs, triIdx, node->triIdx)) {
                return 1;
            }
        } else {
            int res = 0;
            res += contact(bvh, node->l, d_vs, d_fs, d_res);
            res += contact(bvh, node->r, d_vs, d_fs, d_res);
            return res;
        }
    }
    return 0;
}

__host__ __device__
int BVHDenseNode::contact_stack(BVHDenseNode *bvh, int height, vec3f *d_vs, Triangle *d_fs, MyVector *d_res) const {
    int cnt = 0;
    int *stack = new int[height+5];
    int p = 0;
    stack[p++] = 0;

    while (p) {
        int top = stack[--p];
        auto node = &bvh[top];
        if (node->isLeaf() && node->triIdx == triIdx)
            continue;
        if (contact(node)) {
            if (node->isLeaf()) {
                if (MyObj::triContactDetectionCUDA(d_vs, d_fs, triIdx, node->triIdx)) {
                    cnt ++;
//                    d_res->push_back(node->triIdx);
                }
            } else {
                stack[p++] = node->r;
                stack[p++] = node->l;
            }
        }
    }

    delete[] stack;
    return cnt;
}