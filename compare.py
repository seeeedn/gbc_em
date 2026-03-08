def normalize(line):
    """
    Normalize trace lines so small format differences don't matter
    """
    # Keep only left side (before opcode / disasm)
    line = line.split("|", 1)[0]

    # Trim whitespace
    line = line.strip()

    # Normalize PC bank prefix: PC:00:01F2 -> PC:01F2
    line = line.replace("PC:00:", "PC:")

    # Collapse multiple spaces
    line = " ".join(line.split())

    return line


with open("my_log.log") as f1, open("mgba.log") as f2:
    for i, (l1, l2) in enumerate(zip(f1, f2), 1):
        n1 = normalize(l1)
        n2 = normalize(l2)

        if n1 != n2:
            print("❌ DIVERGENCE at instruction", i)
            print("Mine :", n1)
            print("mGBA :", n2)
            break
    else:
        print("✅ No differences found (within compared range)")

