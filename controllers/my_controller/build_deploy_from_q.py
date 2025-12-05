# build_deploy_from_q.py
import json
from collections import defaultdict

ACTIONS = ['向左转', '直行', '向右转']

def build_deploy_from_q(q_file, output_file, locations, max_steps=30):
    with open(q_file, "r", encoding="utf-8") as f:
        q_table = json.load(f)

    # 先把所有状态按 (location, idx) 归类
    grouped = defaultdict(list)  # key: (loc, idx) -> list of (cross_type, q_vals)

    for state_key, q_vals in q_table.items():
        # 兼容这种格式: "Loading Bay A | 28 | 3"
        parts = state_key.split("|")
        if len(parts) != 3:
            continue  # 非预期格式忽略

        loc = parts[0].strip()
        try:
            idx = int(parts[1].strip())
            cross_type = int(parts[2].strip())
        except ValueError:
            continue

        grouped[(loc, idx)].append((cross_type, q_vals))

    lines = []

    for loc in locations:
        if loc == "Loading Start":
            continue

        # 找出该目的地的所有 (idx, cross_type, q_vals)
        entries = []
        for (loc_key, idx), lst in grouped.items():
            if loc_key != loc:
                continue
            # 如果同一个 idx 有多个路口类型/记录，选 max(Q) 最大的那条
            best_ct, best_q = max(lst, key=lambda x: max(x[1]))
            entries.append((idx, best_ct, best_q))

        if not entries:
            print(f"[Q->Deploy] 目的地 {loc} 没有可用状态，生成空决策")
            lines.append(f"Location: {loc}")
            lines.append(f"Best decision: []")
            lines.append("")
            continue

        # 按 idx 排序，相当于路口顺序
        entries.sort(key=lambda x: x[0])

        best_decisions = []

        # 只取前 max_steps 个
        for i, (idx, cross_type, q_vals) in enumerate(entries):
            if i >= max_steps:
                break

            # 根据路口类型限定合法动作
            if cross_type == 1:      # T 字
                valid_actions = [0, 2]       # 左、右
            elif cross_type == 2:    # 十字
                valid_actions = [0, 1, 2]    # 左、直、右
            elif cross_type == 3:    # |- 型
                valid_actions = [1, 2]       # 直、右
            elif cross_type == 4:    # -| 型
                valid_actions = [0, 1]       # 左、直
            else:
                # 未知类型，跳过这个点
                print(f"[Q->Deploy] {loc} idx={idx} 有未知路口类型 {cross_type}，已跳过")
                continue

            # 在合法动作中找 Q 最大的
            best_a = None
            best_q = None
            for a_idx in valid_actions:
                if best_q is None or q_vals[a_idx] > best_q:
                    best_q = q_vals[a_idx]
                    best_a = a_idx

            if best_a is None:
                continue

            best_decisions.append(ACTIONS[best_a])

        lines.append(f"Location: {loc}")
        lines.append(f"Best decision: {best_decisions}")
        lines.append("")

        print(f"[Q->Deploy] {loc}: 生成 {len(best_decisions)} 个决策点")

    with open(output_file, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"已根据 Q 表生成部署决策文件: {output_file}")


if __name__ == "__main__":
    locations = ["Loading Bay A", "Loading Bay B", "Loading Bay C", "Loading Bay D"]
    build_deploy_from_q("q_table.json", "deploy_decision.txt", locations, max_steps=30)
