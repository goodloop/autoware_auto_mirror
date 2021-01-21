
# Create graphviz output to render messages

import csv
import os
import sys


def make_var_name(subcomponent):
    """Make a valid python variable name out of a subcomponent."""
    var_name = subcomponent.replace(' ', '_')
    var_name = var_name.replace('(', '_')
    var_name = var_name.replace(')', '_')
    if var_name[0].isnumeric():
        var_name = '_' + var_name
    return var_name


def to_graphviz(filename):
    submodules = []
    submodule_vars = []
    message_types = []
    topics = []

    with open(filename) as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar='"')

        # skip first line that includes meta info
        next(spamreader)

        for row in spamreader:
            subcomponent, message_type, topic = row
            submodules.append(subcomponent)
            var_name = make_var_name(subcomponent)
            submodule_vars.append(var_name)
            message_types.append(message_type)
            topics.append(topic)

    file_name_root = os.path.splitext(filename)[0]
    with open(file_name_root + ".gv", "w") as output:
        print('digraph {', file=output)
        print('node [shape=box, style=filled]', file=output)
        print(f'subgraph cluster_{file_name_root.replace(" ", "_")}', file=output)
        print('{', file=output)
        print(f'label = "{file_name_root}"', file=output)

        for subcomponent, var in zip(submodules, submodule_vars):
            print(f'{var} [label="{subcomponent}"]', file=output)

        print("}\n", file=output)

        # nodes
        output_nodes = []
        for i, subcomponent in enumerate(submodule_vars):
            output_node = f'{subcomponent}_out'
            # support 1-to-N connections
            if i > 0 and subcomponent == submodule_vars[i-1]:
                output_node = f'{subcomponent}_{i}_out'
            output_nodes.append(output_node)

        print("node [style=rounded]\n", file=output)

        for output_node, message_type, topic in zip(output_nodes, message_types, topics):
            print(f'{output_node}'
                  f'[label=<<TABLE BORDER="0">'
                  f'<TR><TD><b>{message_type}</b></TD></TR>'
                  f'<TR><TD>{topic}</TD></TR>'
                  f'</TABLE>>]', file=output)
        print("", file=output)

        # connections
        for subcomponent, output_node in zip(submodule_vars, output_nodes):
            print(f'{subcomponent} -> {output_node}', file=output)

        print("}", file=output)


if __name__ == '__main__':
    to_graphviz(sys.argv[1])
