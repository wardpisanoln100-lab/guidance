---
name: git-account-switcher-design
description: Git账号切换工具设计规格
---

# Git账号切换工具设计

## 概述

创建一个交互式git账号切换工具，帮助用户在同一台电脑上快速切换4个不同的git账号，用于帮不同人上传代码到同一仓库。

## 需求背景

- **用户场景**：帮4个人每天用各自的账号上传内容到同一仓库（Gitee）
- **切换方式**：交互式菜单，输入1-4选择账号

## 账号配置

| 序号 | 用户名 | 邮箱 |
|------|--------|------|
| 1 | Chaolun | 1989402073@qq.com |
| 2 | 许文轩 | 16622044+xu-wenxuan888@user.noreply.gitee.com |
| 3 | 杜鑫阳1 | 16622068+du-xinyang1@user.noreply.gitee.com |
| 4 | 七度茧 | 16622129+seventh-degree-cocoon@user.noreply.gitee.com |

## 文件结构

```
y8ModelAllControl/
  git-account/
    git-account.sh      # 主脚本（Linux/Mac）
    git-account.bat     # 主脚本（Windows）
    accounts.conf       # 账号配置文件
```

## 功能设计

### 1. 交互式菜单

显示账号列表供用户选择：
```
=== Git账号切换工具 ===
1. Chaolun
2. 许文轩
3. 杜鑫阳1
4. 七度茧
Q. 退出

请选择账号 (1-4): _
```

### 2. 账号切换

选择账号后执行：
```bash
git config --global user.name "用户名"
git config --global user.email "邮箱"
```

### 3. 切换确认

显示当前账号状态：
```
[已切换到: 许文轩]
当前账号: 许文轩 <16622044+xu-wenxuan888@user.noreply.gitee.com>
```

### 4. 查看当前

支持不带参数运行，显示当前账号：
```
当前账号: Chaolun <1989402073@qq.com>
```

## 实现细节

### accounts.conf 格式

```
1,Chaolun,1989402073@qq.com
2,许文轩,16622044+xu-wenxuan888@user.noreply.gitee.com
3,杜鑫阳1,16622068+du-xinyang1@user.noreply.gitee.com
4,七度茧,16622129+seventh-degree-cocoon@user.noreply.gitee.com
```

### 跨平台支持

- **Windows**: `.bat` 文件，使用 `cmd /c`
- **Linux/Mac**: `.sh` 文件，添加执行权限

## 验收标准

1. 交互式菜单正确显示4个账号
2. 输入1-4可切换到对应账号
3. 切换后 git config 正确设置
4. 显示当前账号信息正确
5. 输入无效选项有提示
6. Windows 和 Linux/Mac 都能运行