//
// Created by zhuoc on 2021/11/1.
//

#include "bvh.h"
#include "myobj.h"

#include <iostream>

BVHNode *BVHNode::build(BVHNode **nodes, int n, int depth) {
    if (n == 1) {
        nodes[0]->depth = depth;
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
    if (n == 2) {
        root->l = nodes[0];
        root->r = nodes[1];
        root->l->depth = root->r->depth = depth + 1;
    } else {
        root->l = build(nodes, n / 2, depth + 1);
        root->r = build(nodes + n / 2, n - n/2, depth + 1);
    }

    vec3f a(myfmin(root->l->a.x, root->r->a.x), myfmin(root->l->a.y, root->r->a.y), myfmin(root->l->a.z, root->r->a.z));
    vec3f b(myfmax(root->l->b.x, root->r->b.x), myfmax(root->l->b.y, root->r->b.y), myfmax(root->l->b.z, root->r->b.z));
    root->setBox(a, b);
    root->height = root->l->height + 1;
    root->depth = depth;
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
            cnt++;
            if (obj->triContactDetection(triIdx, node->triIdx)) {
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
