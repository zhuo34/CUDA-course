# Final：自碰撞检测

> 所有源代码托管在 [GitHub][github]。

## 目录

- [Introduction](#Introduction)
  - [任务介绍](#任务介绍)
  - [测试平台](#测试平台)
  - [baseline](#baseline)
- [Methods](#Methods)
  - [BVH 算法](#BVH 算法)
  - [GPU version of BVH](# GPU version of BVH)
- [Results](#Results)
- [Discussion](#Discussion)
  - [算法实现中遇到的问题](#算法实现中遇到的问题)
  - [改进空间](#改进空间)
  - [其它问题](#其它问题)

## Introduction

### 任务介绍

给定一个 `obj` 文件格式的模型，检测模型各三角形之间的自碰撞。

|  模型文件   | 顶点数 | 三角面数 |
| :---------: | :----: | :------: |
| [旗子][obj] | 632674 | 1262274  |

![][obj-img]

### 测试平台

|        CPU        |              GPU               |
| :---------------: | :----------------------------: |
| AMD Ryzen 7 5800H | NVIDIA GeForce RTX 3060 Laptop |

其中 GPU 详细参数如下图所示。

![][gpu]

### baseline

课程提供的 baseline 根据 BVH(Bounding volume hierarchy) 算法检测自碰撞，实现了 CPU 和 GPU 两个版本。baseline 测试如下表。

|               | CPU Version | GPU Version |  加速比   |
| :-----------: | :---------: | :---------: | :-------: |
|   耗时 (ms)   |   733.17    |    28.47    | **25.75** |
| 内存占用 (MB) |      -      |   4324.78   |     -     |

>  注意，由于例程未开放源代码，根据输出信息推测例程算法在 BVH 的基础上使用了法线锥检测对部分碰撞检测进行剔除，并结合空间哈希方法，其原理参考文献 [PSCC: Parallel Self-Collision Culling with Spatial Hashing on GPUs][PSCC]。

## Methods

### BVH 算法

BVH，常译为「包围盒层次结构」，是一种由集合对象组成的**树形结构**。众所周知，在计算机图形学中，求交、碰撞检测等操作开销很大。为了优化这些步骤，BVH 将模型的图元封在一个个「包围盒」中。这些包围盒是以简便计算为目的设计出的，以此通过先对这些包围盒做计算，快速剔除无需进行精细检测的操作。

本项目采用 **AABB 包围盒**。它将图元（本项目为三角形）包含在一个立方体中，在三维空间内，这样的立方体可以通过两个顶点确定。

BVH 的另一关键词是**层次结构**。算法需要根据各个包围盒的空间位置信息，构建出一颗二叉树，它应该具有如下性质：

1. 叶结点由图元组成；
2. 子结点包围盒被父结点完全包围。

在构建出这样的 BVH 后，通过对这棵树进行遍历，就可以实现碰撞检测。本节首先讨论 [BVH 的构建过程](#BVH 树的构建)，接着给出[基于 BVH 的碰撞检测算法](#基于 BVH 的碰撞检测)，最后给出[算法实现](#算法实现)。

#### BVH 树的构建

本项目通过自顶向下的方法构建 BVH。事实上，这是一个并不复杂的递归过程：

1. 将三角形的包围盒按照某一个轴排序，根据中间的三角形坐标将所有三角形分为两部分；
2. 小于该坐标的三角形划分至左子结点，反之划分只右子节点；
3. 递归构建二叉树，直至每个叶结点只包含一个三角形。

#### 基于 BVH 的碰撞检测

在获得 BVH 树后，碰撞检测算法得以优化：

1. 与根节点包围盒判断是否相交；
2. 若相交，则判断与其子结点是否相交；否则结束；
3. 若与叶结点包围盒相交，进行精细的相交检测。

二叉树的高度是 $O(\log N)$，因此理想条件下，自碰撞检测的算法复杂度可从 $O(N^2)$ 降为 $O(N\log N)$，大大优化了时间复杂度。

事实上，BVH 带来的性能提升来自于快速的包围盒之间的检测，以及从 BVH 树中剪支，排除了大量无需检测的三角形。

#### 算法实现

BVH 相关代码在 [`bvh.cu`][bvh-code] 中。

BVH 构建算法实现如下：

```c++
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
```

碰撞检测函数实现如下：

```c++
int BVHNode::contact(BVHNode *node, const MyObj *obj, std::set<std::pair<int, int>> &pairs) {
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
```

### GPU version of BVH

在实现 CPU 版本的 BVH 算法后，GPU 版本需要考虑两个问题：

第一，**数据结构的优化和适配**。CPU 版本中，BVH 树通过递归方法构建，其内存不连续，结点彼此通过指针连接。这样的数据结构在 GPU 版本显然是不合适的。一方面离散化的申请现存是极为低效的；另一方面，为 GPU 上的结点分配正确的指针是一件复杂且极易出 bug 的事。

第二，并行算法。在 GPU 上定义好数据结构后，这方面相对容易。

#### GPU 上的 BVH 数据结构

本项目中对 BVH 树的内存进行整理，将分散的各结点组织到一个长度为 `nodeCnt` 的数组中，并将子结点连接从指针改为数组下标。

```c++
class BVHDenseNode {
public:
    vec3f a, b;
    int triIdx = -1;
    int l = -1;
    int r = -1;
};
```

这样的整理算法同样可以通过递归遍历 BVH 解决，实现如下：

```c++
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
```

其中，数组大小 `nodeCnt` 和每个结点的位置 `idx` 在[构建 BVH][#算法实现] 的同时计算，没有额外开销。

#### 基于 BVH 的 GPU 并行碰撞检测算法

在确定好 BVH 在 GPU 中的数据结构后，对碰撞检测算法稍加修改就可以得到 GPU 并行版本。

```c++
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
```

## Results

在[旗子模型](#任务介绍)上，本项目算法结果如下：

|                   | CPU Version | GPU Version |  加速比  |
| :---------------: | :---------: | :---------: | :------: |
| BVH 构建耗时 (s)  |    2.866    |    3.081    |    -     |
| 碰撞检测耗时 (ms) |   862.05    |   237.77    | **3.78** |
|  内存占用（MB）   |      -      | **151.70**  |          |

调整 block size，耗时变化如下图：

![][block-size]

## Discussion

### 算法实现中遇到的问题

#### GPU 上的递归函数

在修改 GPU 版本的 BVH 遍历算法时，最开始使用了递归函数。结果发现在较小的模型上可以得到正确的结果，但[本项目的旗子模型](#任务介绍)一直返回 0。猜测是由于递归深度太深导致 CUDA block 爆栈了，小模型的 BVH 高度为 16，旗子模型为 20。将递归遍历修改为基于栈的深度遍历后，结果正确。

### 改进空间

#### 并行构建 BVH 树

构建 BVH 过程中的某些步骤也可并行化，例如所有初始化三角形包围盒、排序、左右子结点并行等。

#### 算法改进

本项目是在阅读 [PSCC][PSCC] 这篇论文后完成的，原本目标是进行完全复现。但限于时间及精力有限，加之本人并不是 CG 方向，一些算法复现略有难度，因此算是个半成品。唯一有优势的地方在于，它使用了很少的显存。

此外，本项目没有对内存方面有过多优化，在充分利用共享内存和常量内存后，性能应该会有进一步提升。

### 其它问题

#### 碰撞检测的实际应用

碰撞检测算法在实际应用中放在什么位置，它如何与整个 CG 流程配合？

[obj]:flag-2000-changed/0000_00.obj
[obj-img]: img/flag-obj.png
[PSCC]:https://min-tang.github.io/home/PSCC/
[GPU]:img/gpu.png
[bvh-code]: bvh.cu "bvh 源代码"
[github]: https://github.com/zhuo34/CUDA-course/tree/main/objcd
[block-size]: img/block-size.png