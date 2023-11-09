from javascript import require
import sys


def bundle(file_path, output_file_name):
    bundle_tool = require("./bundleHelper.js")
    json_schema = bundle_tool.bundleHelper(file_path)

    f = open(output_file_name, "a")
    f.write(json_schema)
    f.close()


if __name__ == "__main__":
    bundle(str(sys.argv[1]), str(sys.argv[2]))

# "../schemas/applications/schema/application.schema.json"
