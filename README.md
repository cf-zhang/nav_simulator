# C++语法笔记

## 说明

ReadTheDocs 电子书链接：
> https://cpp-note.readthedocs.io/zh/latest/

网络不好的同学可以看下面的语雀链接（尽量同步更新）：
> https://www.yuque.com/tomocat/txc11h/atgr07

或者 gitee 仓库：
> https://gitee.com/tomocat/CppNote

## 搭建电子书方法

```bash
# optional: 安装 Sphinx
# $pip install -i https://pypi.tuna.tsinghua.edu.cn/simple sphinx

# 安装第三方库
$pip install -r requirements.txt

# 编译
$make html

# 部署
$sphinx-autobuild source build/html
```

## 参考书籍

这本《C++语法笔记》是我阅读下列书籍将相似的知识点归纳总结后而成的，如侵必删。

[1] 《C++ Primer》  王刚 杨巨峰 译

[2] 《Google Style Guide》

[3] 《现代C++教程：高速上手C++11/14/17/20》 欧长坤 译

[4] 《Effective C+ 改善程序与设计的55个具体做法》 侯捷 译

## 编写原则

#### 1. 二八原则

遵循“二八原则”，记录**C++后端开发**中百分之八十以上的知识点，避开各类奇技淫巧。

#### 2. 删繁就简

对于强记忆性的内容简单介绍，比如枯燥的进制转换和少用的语言特性等。

#### 3. 规范编程

以《Google Style Guide》中的C++风格指南为准，跳过了一些不推荐的编程操作，倡导规范化的C++编程。

#### 4. 自完善性

当你在查看本文档时，可能会发现有部分重复的内容，比方说 map 初始化这个知识点同时涉及初始化和关联容器 map，那么它就会同时出现在两个地方，保证每篇独立文档的自完善性。

#### 5. 专题梳理

有些知识点（比如C++初始化规则和关键字）会零零散散地落在多个章节，我会系统梳理这些知识，并以专题的方式沉淀到一篇文章中去。
