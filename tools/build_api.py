"""
mbed SDK
Copyright (c) 2011-2016 ARM Limited

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import re
import tempfile
from types import ListType
from shutil import rmtree
from os.path import join, exists, dirname, basename, abspath, normpath
from os import linesep, remove
from time import time

from tools.utils import mkdir, run_cmd, run_cmd_ext, NotSupportedException,\
    ToolException, InvalidReleaseTargetException
from tools.paths import MBED_CMSIS_PATH, MBED_TARGETS_PATH, MBED_LIBRARIES,\
    MBED_HEADER, MBED_DRIVERS, MBED_PLATFORM, MBED_HAL, MBED_CONFIG_FILE,\
    MBED_LIBRARIES_DRIVERS, MBED_LIBRARIES_PLATFORM, MBED_LIBRARIES_HAL,\
    BUILD_DIR
from tools.targets import TARGET_NAMES, TARGET_MAP
from tools.libraries import Library
from tools.toolchains import TOOLCHAIN_CLASSES
from jinja2 import FileSystemLoader
from jinja2.environment import Environment
from tools.config import Config

RELEASE_VERSIONS = ['2', '5']

def prep_report(report, target_name, toolchain_name, id_name):
    """Setup report keys

    Positional arguments:
    report - the report to fill
    target_name - the target being used
    toolchain_name - the toolchain being used
    id_name - the name of the executable or library being built
    """
    if not target_name in report:
        report[target_name] = {}

    if not toolchain_name in report[target_name]:
        report[target_name][toolchain_name] = {}

    if not id_name in report[target_name][toolchain_name]:
        report[target_name][toolchain_name][id_name] = []

def prep_properties(properties, target_name, toolchain_name, vendor_label):
    """Setup test properties

    Positional arguments:
    properties - the dict to fill
    target_name - the target the test is targeting
    toolchain_name - the toolchain that will compile the test
    vendor_label - the vendor
    """
    if not target_name in properties:
        properties[target_name] = {}

    if not toolchain_name in properties[target_name]:
        properties[target_name][toolchain_name] = {}

    properties[target_name][toolchain_name]["target"] = target_name
    properties[target_name][toolchain_name]["vendor"] = vendor_label
    properties[target_name][toolchain_name]["toolchain"] = toolchain_name

def create_result(target_name, toolchain_name, id_name, description):
    """Create a result dictionary

    Positional arguments:
    target_name - the target being built for
    toolchain_name - the toolchain doing the building
    id_name - the name of the executable or library being built
    description - a human readable description of what's going on
    """
    cur_result = {}
    cur_result["target_name"] = target_name
    cur_result["toolchain_name"] = toolchain_name
    cur_result["id"] = id_name
    cur_result["description"] = description
    cur_result["elapsed_time"] = 0
    cur_result["output"] = ""

    return cur_result

def add_result_to_report(report, result):
    """Add a single result to a report dictionary

    Positional arguments:
    report - the report to append to
    result - the result to append
    """
    target = result["target_name"]
    toolchain = result["toolchain_name"]
    id_name = result['id']
    result_wrap = {0: result}
    report[target][toolchain][id_name].append(result_wrap)

def get_config(src_paths, target, toolchain_name):
    """Get the configuration object for a target-toolchain combination

    Positional arguments:
    src_paths - paths to scan for the configuration files
    target - the device we are building for
    toolchain_name - the string that identifies the build tools
    """
    # Convert src_paths to a list if needed
    if type(src_paths) != ListType:
        src_paths = [src_paths]

    # Pass all params to the unified prepare_resources()
    toolchain = prepare_toolchain(src_paths, target, toolchain_name)

    # Scan src_path for config files
    resources = toolchain.scan_resources(src_paths[0])
    for path in src_paths[1:]:
        resources.add(toolchain.scan_resources(path))

    # Update configuration files until added features creates no changes
    prev_features = set()
    while True:
        # Update the configuration with any .json files found while scanning
        toolchain.config.add_config_files(resources.json_files)

        # Add features while we find new ones
        features = toolchain.config.get_features()
        if features == prev_features:
            break

        for feature in features:
            if feature in resources.features:
                resources += resources.features[feature]

        prev_features = features
    toolchain.config.validate_config()

    cfg, macros = toolchain.config.get_config_data()
    features = toolchain.config.get_features()
    return cfg, macros, features

def is_official_target(target_name, version):
    """ Returns True, None if a target is part of the official release for the
    given version. Return False, 'reason' if a target is not part of the
    official release for the given version.

    Positional arguments:
    target_name - Name if the target (ex. 'K64F')
    version - The release version string. Should be a string contained within
              RELEASE_VERSIONS
    """

    result = True
    reason = None
    target = TARGET_MAP[target_name]

    if hasattr(target, 'release_versions') \
       and version in target.release_versions:
        if version == '2':
            # For version 2, either ARM or uARM toolchain support is required
            required_toolchains = set(['ARM', 'uARM'])

            if not len(required_toolchains.intersection(
                    set(target.supported_toolchains))) > 0:
                result = False
                reason = ("Target '%s' must support " % target.name) + \
                    ("one of the folowing toolchains to be included in the") + \
                    ((" mbed 2.0 official release: %s" + linesep) %
                     ", ".join(required_toolchains)) + \
                    ("Currently it is only configured to support the ") + \
                    ("following toolchains: %s" %
                     ", ".join(target.supported_toolchains))

        elif version == '5':
            # For version 5, ARM, GCC_ARM, and IAR toolchain support is required
            required_toolchains = set(['ARM', 'GCC_ARM', 'IAR'])
            required_toolchains_sorted = list(required_toolchains)
            required_toolchains_sorted.sort()
            supported_toolchains = set(target.supported_toolchains)
            supported_toolchains_sorted = list(supported_toolchains)
            supported_toolchains_sorted.sort()

            if not required_toolchains.issubset(supported_toolchains):
                result = False
                reason = ("Target '%s' must support " % target.name) + \
                    ("ALL of the folowing toolchains to be included in the") + \
                    ((" mbed OS 5.0 official release: %s" + linesep) %
                     ", ".join(required_toolchains_sorted)) + \
                    ("Currently it is only configured to support the ") + \
                    ("following toolchains: %s" %
                     ", ".join(supported_toolchains_sorted))

            elif not target.default_lib == 'std':
                result = False
                reason = ("Target '%s' must set the " % target.name) + \
                    ("'default_lib' to 'std' to be included in the ") + \
                    ("mbed OS 5.0 official release." + linesep) + \
                    ("Currently it is set to '%s'" % target.default_lib)

        else:
            result = False
            reason = ("Target '%s' has set an invalid release version of '%s'" %
                      version) + \
                ("Please choose from the following release versions: %s" %
                 ', '.join(RELEASE_VERSIONS))

    else:
        result = False
        if not hasattr(target, 'release_versions'):
            reason = "Target '%s' " % target.name
            reason += "does not have the 'release_versions' key set"
        elif not version in target.release_versions:
            reason = "Target '%s' does not contain the version '%s' " % \
                     (target.name, version)
            reason += "in its 'release_versions' key"

    return result, reason

def transform_release_toolchains(toolchains, version):
    """ Given a list of toolchains and a release version, return a list of
    only the supported toolchains for that release

    Positional arguments:
    toolchains - The list of toolchains
    version - The release version string. Should be a string contained within
              RELEASE_VERSIONS
    """
    if version == '5':
        return ['ARM', 'GCC_ARM', 'IAR']
    else:
        return toolchains


def get_mbed_official_release(version):
    """ Given a release version string, return a tuple that contains a target
    and the supported toolchains for that release.
    Ex. Given '2', return (('LPC1768', ('ARM', 'GCC_ARM')),
                           ('K64F', ('ARM', 'GCC_ARM')), ...)

    Positional arguments:
    version - The version string. Should be a string contained within
              RELEASE_VERSIONS
    """

    mbed_official_release = (
        tuple(
            tuple(
                [
                    TARGET_MAP[target].name,
                    tuple(transform_release_toolchains(
                        TARGET_MAP[target].supported_toolchains, version))
                ]
            ) for target in TARGET_NAMES \
            if (hasattr(TARGET_MAP[target], 'release_versions')
                and version in TARGET_MAP[target].release_versions)
        )
    )

    for target in mbed_official_release:
        is_official, reason = is_official_target(target[0], version)

        if not is_official:
            raise InvalidReleaseTargetException(reason)

    return mbed_official_release


def prepare_toolchain(src_paths, target, toolchain_name,
                      macros=None, clean=False, jobs=1,
                      notify=None, silent=False, verbose=False,
                      extra_verbose=False, config=None,
                      app_config=None, build_profile=None):
    """ Prepares resource related objects - toolchain, target, config

    Positional arguments:
    src_paths - the paths to source directories
    target - ['LPC1768', 'LPC11U24', 'LPC2368', etc.]
    toolchain_name - ['ARM', 'uARM', 'GCC_ARM', 'GCC_CR']

    Keyword arguments:
    macros - additional macros
    clean - Rebuild everything if True
    jobs - how many compilers we can run at once
    notify - Notify function for logs
    silent - suppress printing of progress indicators
    verbose - Write the actual tools command lines used if True
    extra_verbose - even more output!
    config - a Config object to use instead of creating one
    app_config - location of a chosen mbed_app.json file
    build_profile - a dict of flags that will be passed to the compiler
    """

    # We need to remove all paths which are repeated to avoid
    # multiple compilations and linking with the same objects
    src_paths = [src_paths[0]] + list(set(src_paths[1:]))

    # If the configuration object was not yet created, create it now
    config = config or Config(target, src_paths, app_config=app_config)
    target = config.target

    # Toolchain instance
    try:
        toolchain = TOOLCHAIN_CLASSES[toolchain_name](
            target, notify, macros, silent,
            extra_verbose=extra_verbose, build_profile=build_profile)
    except KeyError:
        raise KeyError("Toolchain %s not supported" % toolchain_name)

    toolchain.config = config
    toolchain.jobs = jobs
    toolchain.build_all = clean
    toolchain.VERBOSE = verbose

    return toolchain

def scan_resources(src_paths, toolchain, dependencies_paths=None,
                   inc_dirs=None, base_path=None):
    """ Scan resources using initialized toolcain

    Positional arguments
    src_paths - the paths to source directories
    toolchain - valid toolchain object
    dependencies_paths - dependency paths that we should scan for include dirs
    inc_dirs - additional include directories which should be added to
               the scanner resources
    """

    # Scan src_path
    resources = toolchain.scan_resources(src_paths[0], base_path=base_path)
    for path in src_paths[1:]:
        resources.add(toolchain.scan_resources(path, base_path=base_path))

    # Scan dependency paths for include dirs
    if dependencies_paths is not None:
        for path in dependencies_paths:
            lib_resources = toolchain.scan_resources(path)
            resources.inc_dirs.extend(lib_resources.inc_dirs)

    # Add additional include directories if passed
    if inc_dirs:
        if type(inc_dirs) == ListType:
            resources.inc_dirs.extend(inc_dirs)
        else:
            resources.inc_dirs.append(inc_dirs)

    # Load resources into the config system which might expand/modify resources
    # based on config data
    resources = toolchain.config.load_resources(resources)

    # Set the toolchain's configuration data
    toolchain.set_config_data(toolchain.config.get_config_data())

    return resources

def build_project(src_paths, build_path, target, toolchain_name,
                  libraries_paths=None, linker_script=None,
                  clean=False, notify=None, verbose=False, name=None,
                  macros=None, inc_dirs=None, jobs=1, silent=False,
                  report=None, properties=None, project_id=None,
                  project_description=None, extra_verbose=False, config=None,
                  app_config=None, build_profile=None):
    """ Build a project. A project may be a test or a user program.

    Positional arguments:
    src_paths - a path or list of paths that contain all files needed to build
                the project
    build_path - the directory where all of the object files will be placed
    target - the MCU or board that the project will compile for
    toolchain_name - the name of the build tools

    Keyword arguments:
    libraries_paths - The location of libraries to include when linking
    linker_script - the file that drives the linker to do it's job
    clean - Rebuild everything if True
    notify - Notify function for logs
    verbose - Write the actual tools command lines used if True
    name - the name of the project
    macros - additional macros
    inc_dirs - additional directories where include files may be found
    jobs - how many compilers we can run at once
    silent - suppress printing of progress indicators
    report - a dict where a result may be appended
    properties - UUUUHHHHH beats me
    project_id - the name put in the report
    project_description - the human-readable version of what this thing does
    extra_verbose - even more output!
    config - a Config object to use instead of creating one
    app_config - location of a chosen mbed_app.json file
    build_profile - a dict of flags that will be passed to the compiler
    """

    # Convert src_path to a list if needed
    if type(src_paths) != ListType:
        src_paths = [src_paths]
    # Extend src_paths wiht libraries_paths
    if libraries_paths is not None:
        src_paths.extend(libraries_paths)
        inc_dirs.extend(map(dirname, libraries_paths))

    # Build Directory
    if clean and exists(build_path):
        rmtree(build_path)
    mkdir(build_path)

    # Pass all params to the unified prepare_toolchain()
    toolchain = prepare_toolchain(
        src_paths, target, toolchain_name, macros=macros, clean=clean,
        jobs=jobs, notify=notify, silent=silent, verbose=verbose,
        extra_verbose=extra_verbose, config=config, app_config=app_config,
        build_profile=build_profile)

    # The first path will give the name to the library
    if name is None:
        name = basename(normpath(abspath(src_paths[0])))
    toolchain.info("Building project %s (%s, %s)" %
                   (name, toolchain.target.name, toolchain_name))

    # Initialize reporting
    if report != None:
        start = time()
        # If project_id is specified, use that over the default name
        id_name = project_id.upper() if project_id else name.upper()
        description = project_description if project_description else name
        vendor_label = toolchain.target.extra_labels[0]
        prep_report(report, toolchain.target.name, toolchain_name, id_name)
        cur_result = create_result(toolchain.target.name, toolchain_name,
                                   id_name, description)
        if properties != None:
            prep_properties(properties, toolchain.target.name, toolchain_name,
                            vendor_label)

    try:
        # Call unified scan_resources
        resources = scan_resources(src_paths, toolchain, inc_dirs=inc_dirs)

        # Change linker script if specified
        if linker_script is not None:
            resources.linker_script = linker_script

        # Compile Sources
        objects = toolchain.compile_sources(resources, build_path,
                                            resources.inc_dirs)
        resources.objects.extend(objects)

        # Link Program
        res, _ = toolchain.link_program(resources, build_path, name)

        memap_instance = getattr(toolchain, 'memap_instance', None)
        memap_table = ''
        if memap_instance:
            # Write output to stdout in text (pretty table) format
            memap_table = memap_instance.generate_output('table')

            if not silent:
                print memap_table

            # Write output to file in JSON format
            map_out = join(build_path, name + "_map.json")
            memap_instance.generate_output('json', map_out)

            # Write output to file in CSV format for the CI
            map_csv = join(build_path, name + "_map.csv")
            memap_instance.generate_output('csv-ci', map_csv)

        resources.detect_duplicates(toolchain)

        if report != None:
            end = time()
            cur_result["elapsed_time"] = end - start
            cur_result["output"] = toolchain.get_output() + memap_table
            cur_result["result"] = "OK"
            cur_result["memory_usage"] = toolchain.map_outputs

            add_result_to_report(report, cur_result)

        return res

    except Exception as exc:
        if report != None:
            end = time()

            if isinstance(exc, NotSupportedException):
                cur_result["result"] = "NOT_SUPPORTED"
            else:
                cur_result["result"] = "FAIL"

            cur_result["elapsed_time"] = end - start

            toolchain_output = toolchain.get_output()
            if toolchain_output:
                cur_result["output"] += toolchain_output

            add_result_to_report(report, cur_result)

        # Let Exception propagate
        raise

def build_library(src_paths, build_path, target, toolchain_name,
                  dependencies_paths=None, name=None, clean=False,
                  archive=True, notify=None, verbose=False, macros=None,
                  inc_dirs=None, jobs=1, silent=False, report=None,
                  properties=None, extra_verbose=False, project_id=None,
                  remove_config_header_file=False, app_config=None,
                  build_profile=None):
    """ Build a library

    Positional arguments:
    src_paths - a path or list of paths that contain all files needed to build
                the library
    build_path - the directory where all of the object files will be placed
    target - the MCU or board that the project will compile for
    toolchain_name - the name of the build tools

    Keyword arguments:
    dependencies_paths - The location of libraries to include when linking
    name - the name of the library
    clean - Rebuild everything if True
    archive - whether the library will create an archive file
    notify - Notify function for logs
    verbose - Write the actual tools command lines used if True
    macros - additional macros
    inc_dirs - additional directories where include files may be found
    jobs - how many compilers we can run at once
    silent - suppress printing of progress indicators
    report - a dict where a result may be appended
    properties - UUUUHHHHH beats me
    extra_verbose - even more output!
    project_id - the name that goes in the report
    remove_config_header_file - delete config header file when done building
    app_config - location of a chosen mbed_app.json file
    build_profile - a dict of flags that will be passed to the compiler
    """

    # Convert src_path to a list if needed
    if type(src_paths) != ListType:
        src_paths = [src_paths]

    # Build path
    if archive:
        # Use temp path when building archive
        tmp_path = join(build_path, '.temp')
        mkdir(tmp_path)
    else:
        tmp_path = build_path

    # Clean the build directory
    if clean and exists(tmp_path):
        rmtree(tmp_path)
    mkdir(tmp_path)

    # Pass all params to the unified prepare_toolchain()
    toolchain = prepare_toolchain(
        src_paths, target, toolchain_name, macros=macros, clean=clean,
        jobs=jobs, notify=notify, silent=silent, verbose=verbose,
        extra_verbose=extra_verbose, app_config=app_config,
        build_profile=build_profile)

    # The first path will give the name to the library
    if name is None:
        name = basename(normpath(abspath(src_paths[0])))
    toolchain.info("Building library %s (%s, %s)" %
                   (name, toolchain.target.name, toolchain_name))

    # Initialize reporting
    if report != None:
        start = time()
        # If project_id is specified, use that over the default name
        id_name = project_id.upper() if project_id else name.upper()
        description = name
        vendor_label = toolchain.target.extra_labels[0]
        prep_report(report, toolchain.target.name, toolchain_name, id_name)
        cur_result = create_result(toolchain.target.name, toolchain_name,
                                   id_name, description)
        if properties != None:
            prep_properties(properties, toolchain.target.name, toolchain_name,
                            vendor_label)

    for src_path in src_paths:
        if not exists(src_path):
            error_msg = "The library source folder does not exist: %s", src_path
            if report != None:
                cur_result["output"] = error_msg
                cur_result["result"] = "FAIL"
                add_result_to_report(report, cur_result)
            raise Exception(error_msg)

    try:
        # Call unified scan_resources
        resources = scan_resources(src_paths, toolchain,
                                   dependencies_paths=dependencies_paths,
                                   inc_dirs=inc_dirs)


        # Copy headers, objects and static libraries - all files needed for
        # static lib
        toolchain.copy_files(resources.headers, build_path, resources=resources)
        toolchain.copy_files(resources.objects, build_path, resources=resources)
        toolchain.copy_files(resources.libraries, build_path,
                             resources=resources)
        toolchain.copy_files(resources.json_files, build_path,
                             resources=resources)
        if resources.linker_script:
            toolchain.copy_files(resources.linker_script, build_path,
                                 resources=resources)

        if resources.hex_files:
            toolchain.copy_files(resources.hex_files, build_path,
                                 resources=resources)

        # Compile Sources
        objects = toolchain.compile_sources(resources, abspath(tmp_path),
                                            resources.inc_dirs)
        resources.objects.extend(objects)

        if archive:
            toolchain.build_library(objects, build_path, name)

        if remove_config_header_file:
            config_header_path = toolchain.get_config_header()
            if config_header_path:
                remove(config_header_path)

        if report != None:
            end = time()
            cur_result["elapsed_time"] = end - start
            cur_result["output"] = toolchain.get_output()
            cur_result["result"] = "OK"


            add_result_to_report(report, cur_result)
        return True

    except Exception as exc:
        if report != None:
            end = time()

            if isinstance(exc, ToolException):
                cur_result["result"] = "FAIL"
            elif isinstance(exc, NotSupportedException):
                cur_result["result"] = "NOT_SUPPORTED"

            cur_result["elapsed_time"] = end - start

            toolchain_output = toolchain.get_output()
            if toolchain_output:
                cur_result["output"] += toolchain_output

            add_result_to_report(report, cur_result)

        # Let Exception propagate
        raise

######################
### Legacy methods ###
######################

def build_lib(lib_id, target, toolchain_name, verbose=False,
              clean=False, macros=None, notify=None, jobs=1, silent=False,
              report=None, properties=None, extra_verbose=False,
              build_profile=None):
    """ Legacy method for building mbed libraries

    Positional arguments:
    lib_id - the library's unique identifier
    target - the MCU or board that the project will compile for
    toolchain_name - the name of the build tools

    Keyword arguments:
    clean - Rebuild everything if True
    verbose - Write the actual tools command lines used if True
    macros - additional macros
    notify - Notify function for logs
    jobs - how many compilers we can run at once
    silent - suppress printing of progress indicators
    report - a dict where a result may be appended
    properties - UUUUHHHHH beats me
    extra_verbose - even more output!
    build_profile - a dict of flags that will be passed to the compiler
    """
    lib = Library(lib_id)
    if not lib.is_supported(target, toolchain_name):
        print('Library "%s" is not yet supported on target %s with toolchain %s'
              % (lib_id, target.name, toolchain_name))
        return False

    # We need to combine macros from parameter list with macros from library
    # definition
    lib_macros = lib.macros if lib.macros else []
    if macros:
        macros.extend(lib_macros)
    else:
        macros = lib_macros

    src_paths = lib.source_dir
    build_path = lib.build_dir
    dependencies_paths = lib.dependencies
    inc_dirs = lib.inc_dirs
    inc_dirs_ext = lib.inc_dirs_ext

    if type(src_paths) != ListType:
        src_paths = [src_paths]

    # The first path will give the name to the library
    name = basename(src_paths[0])

    if report != None:
        start = time()
        id_name = name.upper()
        description = name
        vendor_label = target.extra_labels[0]
        cur_result = None
        prep_report(report, target.name, toolchain_name, id_name)
        cur_result = create_result(target.name, toolchain_name, id_name,
                                   description)

        if properties != None:
            prep_properties(properties, target.name, toolchain_name,
                            vendor_label)

    for src_path in src_paths:
        if not exists(src_path):
            error_msg = "The library source folder does not exist: %s", src_path

            if report != None:
                cur_result["output"] = error_msg
                cur_result["result"] = "FAIL"
                add_result_to_report(report, cur_result)

            raise Exception(error_msg)

    try:
        # Toolchain instance
        toolchain = TOOLCHAIN_CLASSES[toolchain_name](
            target, macros=macros, notify=notify, silent=silent,
            extra_verbose=extra_verbose, build_profile=build_profile)
        toolchain.VERBOSE = verbose
        toolchain.jobs = jobs
        toolchain.build_all = clean

        toolchain.info("Building library %s (%s, %s)" %
                       (name.upper(), target.name, toolchain_name))

        # Take into account the library configuration (MBED_CONFIG_FILE)
        config = Config(target)
        toolchain.config = config
        config.add_config_files([MBED_CONFIG_FILE])

        # Scan Resources
        resources = []
        for src_path in src_paths:
            resources.append(toolchain.scan_resources(src_path))

        # Add extra include directories / files which are required by library
        # This files usually are not in the same directory as source files so
        # previous scan will not include them
        if inc_dirs_ext is not None:
            for inc_ext in inc_dirs_ext:
                resources.append(toolchain.scan_resources(inc_ext))

        # Dependencies Include Paths
        dependencies_include_dir = []
        if dependencies_paths is not None:
            for path in dependencies_paths:
                lib_resources = toolchain.scan_resources(path)
                dependencies_include_dir.extend(lib_resources.inc_dirs)
                dependencies_include_dir.extend(map(dirname, lib_resources.inc_dirs))

        if inc_dirs:
            dependencies_include_dir.extend(inc_dirs)

        # Add other discovered configuration data to the configuration object
        for res in resources:
            config.load_resources(res)
        toolchain.set_config_data(toolchain.config.get_config_data())

        # Create the desired build directory structure
        bin_path = join(build_path, toolchain.obj_path)
        mkdir(bin_path)
        tmp_path = join(build_path, '.temp', toolchain.obj_path)
        mkdir(tmp_path)

        # Copy Headers
        for resource in resources:
            toolchain.copy_files(resource.headers, build_path,
                                 resources=resource)

        dependencies_include_dir.extend(
            toolchain.scan_resources(build_path).inc_dirs)

        # Compile Sources
        objects = []
        for resource in resources:
            objects.extend(toolchain.compile_sources(resource, tmp_path,
                                                     dependencies_include_dir))

        needed_update = toolchain.build_library(objects, bin_path, name)

        if report != None and needed_update:
            end = time()
            cur_result["elapsed_time"] = end - start
            cur_result["output"] = toolchain.get_output()
            cur_result["result"] = "OK"

            add_result_to_report(report, cur_result)
        return True

    except Exception:
        if report != None:
            end = time()
            cur_result["result"] = "FAIL"
            cur_result["elapsed_time"] = end - start

            toolchain_output = toolchain.get_output()
            if toolchain_output:
                cur_result["output"] += toolchain_output

            add_result_to_report(report, cur_result)

        # Let Exception propagate
        raise

# We do have unique legacy conventions about how we build and package the mbed
# library
def build_mbed_libs(target, toolchain_name, verbose=False,
                    clean=False, macros=None, notify=None, jobs=1, silent=False,
                    report=None, properties=None, extra_verbose=False,
                    build_profile=None):
    """ Function returns True is library was built and false if building was
    skipped

    Positional arguments:
    target - the MCU or board that the project will compile for
    toolchain_name - the name of the build tools

    Keyword arguments:
    verbose - Write the actual tools command lines used if True
    clean - Rebuild everything if True
    macros - additional macros
    notify - Notify function for logs
    jobs - how many compilers we can run at once
    silent - suppress printing of progress indicators
    report - a dict where a result may be appended
    properties - UUUUHHHHH beats me
    extra_verbose - even more output!
    build_profile - a dict of flags that will be passed to the compiler
    """

    if report != None:
        start = time()
        id_name = "MBED"
        description = "mbed SDK"
        vendor_label = target.extra_labels[0]
        cur_result = None
        prep_report(report, target.name, toolchain_name, id_name)
        cur_result = create_result(target.name, toolchain_name, id_name,
                                   description)

        if properties != None:
            prep_properties(properties, target.name, toolchain_name,
                            vendor_label)

    # Check toolchain support
    if toolchain_name not in target.supported_toolchains:
        supported_toolchains_text = ", ".join(target.supported_toolchains)
        print('%s target is not yet supported by toolchain %s' %
              (target.name, toolchain_name))
        print('%s target supports %s toolchain%s' %
              (target.name, supported_toolchains_text, 's'
               if len(target.supported_toolchains) > 1 else ''))

        if report != None:
            cur_result["result"] = "SKIP"
            add_result_to_report(report, cur_result)

        return False

    try:
        # Toolchain
        toolchain = TOOLCHAIN_CLASSES[toolchain_name](
            target, macros=macros, notify=notify, silent=silent,
            extra_verbose=extra_verbose, build_profile=build_profile)
        toolchain.VERBOSE = verbose
        toolchain.jobs = jobs
        toolchain.build_all = clean

        # Take into account the library configuration (MBED_CONFIG_FILE)
        config = Config(target)
        toolchain.config = config
        config.add_config_files([MBED_CONFIG_FILE])
        toolchain.set_config_data(toolchain.config.get_config_data())

        # Source and Build Paths
        build_target = join(MBED_LIBRARIES, "TARGET_" + target.name)
        build_toolchain = join(build_target, "TOOLCHAIN_" + toolchain.name)
        mkdir(build_toolchain)

        tmp_path = join(MBED_LIBRARIES, '.temp', toolchain.obj_path)
        mkdir(tmp_path)

        # CMSIS
        toolchain.info("Building library %s (%s, %s)" %
                       ('CMSIS', target.name, toolchain_name))
        cmsis_src = MBED_CMSIS_PATH
        resources = toolchain.scan_resources(cmsis_src)

        toolchain.copy_files(resources.headers, build_target)
        toolchain.copy_files(resources.linker_script, build_toolchain)
        toolchain.copy_files(resources.bin_files, build_toolchain)

        objects = toolchain.compile_sources(resources, tmp_path)
        toolchain.copy_files(objects, build_toolchain)

        # mbed
        toolchain.info("Building library %s (%s, %s)" %
                       ('MBED', target.name, toolchain_name))

        # Common Headers
        toolchain.copy_files([MBED_HEADER], MBED_LIBRARIES)
        library_incdirs = [dirname(MBED_LIBRARIES), MBED_LIBRARIES]

        for dir, dest in [(MBED_DRIVERS, MBED_LIBRARIES_DRIVERS),
                          (MBED_PLATFORM, MBED_LIBRARIES_PLATFORM),
                          (MBED_HAL, MBED_LIBRARIES_HAL)]:
            resources = toolchain.scan_resources(dir)
            toolchain.copy_files(resources.headers, dest)
            library_incdirs.append(dest)

        # Target specific sources
        hal_src = MBED_TARGETS_PATH
        hal_implementation = toolchain.scan_resources(hal_src)
        toolchain.copy_files(hal_implementation.headers +
                             hal_implementation.hex_files +
                             hal_implementation.libraries +
                             [MBED_CONFIG_FILE],
                             build_target, resources=hal_implementation)
        toolchain.copy_files(hal_implementation.linker_script, build_toolchain)
        toolchain.copy_files(hal_implementation.bin_files, build_toolchain)
        incdirs = toolchain.scan_resources(build_target).inc_dirs
        objects = toolchain.compile_sources(hal_implementation, tmp_path,
                                            library_incdirs + incdirs)
        toolchain.copy_files(objects, build_toolchain)

        # Common Sources
        mbed_resources = None
        for dir in [MBED_DRIVERS, MBED_PLATFORM, MBED_HAL]:
            mbed_resources += toolchain.scan_resources(dir)

        objects = toolchain.compile_sources(mbed_resources, tmp_path,
                                            library_incdirs + incdirs)

        # A number of compiled files need to be copied as objects as opposed to
        # way the linker search for symbols in archives. These are:
        #   - retarget.o: to make sure that the C standard lib symbols get
        #                 overridden
        #   - board.o: mbed_die is weak
        #   - mbed_overrides.o: this contains platform overrides of various
        #                       weak SDK functions
        separate_names, separate_objects = ['retarget.o', 'board.o',
                                            'mbed_overrides.o'], []

        for obj in objects:
            for name in separate_names:
                if obj.endswith(name):
                    separate_objects.append(obj)

        for obj in separate_objects:
            objects.remove(obj)

        toolchain.build_library(objects, build_toolchain, "mbed")

        for obj in separate_objects:
            toolchain.copy_files(obj, build_toolchain)

        if report != None:
            end = time()
            cur_result["elapsed_time"] = end - start
            cur_result["output"] = toolchain.get_output()
            cur_result["result"] = "OK"

            add_result_to_report(report, cur_result)

        return True

    except Exception as exc:
        if report != None:
            end = time()
            cur_result["result"] = "FAIL"
            cur_result["elapsed_time"] = end - start

            toolchain_output = toolchain.get_output()
            if toolchain_output:
                cur_result["output"] += toolchain_output

            cur_result["output"] += str(exc)

            add_result_to_report(report, cur_result)

        # Let Exception propagate
        raise


def get_unique_supported_toolchains(release_targets=None):
    """ Get list of all unique toolchains supported by targets

    Keyword arguments:
    release_targets - tuple structure returned from get_mbed_official_release().
                      If release_targets is not specified, then it queries all
                      known targets
    """
    unique_supported_toolchains = []

    if not release_targets:
        for target in TARGET_NAMES:
            for toolchain in TARGET_MAP[target].supported_toolchains:
                if toolchain not in unique_supported_toolchains:
                    unique_supported_toolchains.append(toolchain)
    else:
        for target in release_targets:
            for toolchain in target[1]:
                if toolchain not in unique_supported_toolchains:
                    unique_supported_toolchains.append(toolchain)

    return unique_supported_toolchains


def mcu_toolchain_matrix(verbose_html=False, platform_filter=None,
                         release_version='5'):
    """  Shows target map using prettytable

    Keyword arguments:
    verbose_html - emit html instead of a simple table
    platform_filter - remove results that match the string
    release_version - get the matrix for this major version number
    """
    # Only use it in this function so building works without extra modules
    from prettytable import PrettyTable

    if isinstance(release_version, basestring):
        # Force release_version to lowercase if it is a string
        release_version = release_version.lower()
    else:
        # Otherwise default to printing all known targets and toolchains
        release_version = 'all'


    version_release_targets = {}
    version_release_target_names = {}

    for version in RELEASE_VERSIONS:
        version_release_targets[version] = get_mbed_official_release(version)
        version_release_target_names[version] = [x[0] for x in
                                                 version_release_targets[
                                                     version]]

    if release_version in RELEASE_VERSIONS:
        release_targets = version_release_targets[release_version]
    else:
        release_targets = None

    unique_supported_toolchains = get_unique_supported_toolchains(
        release_targets)
    prepend_columns = ["Target"] + ["mbed OS %s" % x for x in RELEASE_VERSIONS]

    # All tests status table print
    columns = prepend_columns + unique_supported_toolchains
    table_printer = PrettyTable(columns)
    # Align table
    for col in columns:
        table_printer.align[col] = "c"
    table_printer.align["Target"] = "l"

    perm_counter = 0
    target_counter = 0

    target_names = []

    if release_targets:
        target_names = [x[0] for x in release_targets]
    else:
        target_names = TARGET_NAMES

    for target in sorted(target_names):
        if platform_filter is not None:
            # FIlter out platforms using regex
            if re.search(platform_filter, target) is None:
                continue
        target_counter += 1

        row = [target]  # First column is platform name

        for version in RELEASE_VERSIONS:
            if target in version_release_target_names[version]:
                text = "Supported"
            else:
                text = "-"
            row.append(text)

        for unique_toolchain in unique_supported_toolchains:
            if unique_toolchain in TARGET_MAP[target].supported_toolchains:
                text = "Supported"
                perm_counter += 1
            else:
                text = "-"

            row.append(text)
        table_printer.add_row(row)

    result = table_printer.get_html_string() if verbose_html \
             else table_printer.get_string()
    result += "\n"
    result += "Supported targets: %d\n"% (target_counter)
    if target_counter == 1:
        result += "Supported toolchains: %d"% (perm_counter)
    return result


def get_target_supported_toolchains(target):
    """ Returns target supported toolchains list

    Positional arguments:
    target - the target to get the supported toolchains of
    """
    return TARGET_MAP[target].supported_toolchains if target in TARGET_MAP \
        else None


def static_analysis_scan(target, toolchain_name, cppcheck_cmd,
                         cppcheck_msg_format, verbose=False,
                         clean=False, macros=None, notify=None, jobs=1,
                         extra_verbose=False, build_profile=None):
    """Perform static analysis on a target and toolchain combination

    Positional arguments:
    target - the target to fake the build for
    toolchain_name - pretend you would compile with this toolchain
    cppcheck_cmd - the command used to do static analysis
    cppcheck_msg_format - the format of the check messages

    Keyword arguments:
    verbose - more printing!
    clean - start from a clean slate
    macros - extra macros to compile with
    notify - the notification event handling function
    jobs - number of commands to run at once
    extra_verbose - even moar printing
    build_profile - a dict of flags that will be passed to the compiler
    """
    # Toolchain
    toolchain = TOOLCHAIN_CLASSES[toolchain_name](target, macros=macros,
                                                  notify=notify,
                                                  extra_verbose=extra_verbose,
                                                  build_profile=build_profile)
    toolchain.VERBOSE = verbose
    toolchain.jobs = jobs
    toolchain.build_all = clean

    # Source and Build Paths
    build_target = join(MBED_LIBRARIES, "TARGET_" + target.name)
    build_toolchain = join(build_target, "TOOLCHAIN_" + toolchain.name)
    mkdir(build_toolchain)

    tmp_path = join(MBED_LIBRARIES, '.temp', toolchain.obj_path)
    mkdir(tmp_path)

    # CMSIS
    toolchain.info("Static analysis for %s (%s, %s)" %
                   ('CMSIS', target.name, toolchain_name))
    cmsis_src = MBED_CMSIS_PATH
    resources = toolchain.scan_resources(cmsis_src)

    # Copy files before analysis
    toolchain.copy_files(resources.headers, build_target)
    toolchain.copy_files(resources.linker_script, build_toolchain)

    # Gather include paths, c, cpp sources and macros to transfer to cppcheck
    # command line
    includes = ["-I%s"% i for i in resources.inc_dirs]
    includes.append("-I%s"% str(build_target))
    c_sources = " ".join(resources.c_sources)
    cpp_sources = " ".join(resources.cpp_sources)
    macros = ["-D%s"% s for s in toolchain.get_symbols() + toolchain.macros]

    includes = [inc.strip() for inc in includes]
    macros = [mac.strip() for mac in macros]

    check_cmd = cppcheck_cmd
    check_cmd += cppcheck_msg_format
    check_cmd += includes
    check_cmd += macros

    # We need to pass some params via file to avoid "command line too long in
    # some OSs"
    tmp_file = tempfile.NamedTemporaryFile(delete=False)
    tmp_file.writelines(line + '\n' for line in c_sources.split())
    tmp_file.writelines(line + '\n' for line in cpp_sources.split())
    tmp_file.close()
    check_cmd += ["--file-list=%s"% tmp_file.name]

    _stdout, _stderr, _ = run_cmd(check_cmd)
    if verbose:
        print _stdout
    print _stderr

    # =========================================================================

    # MBED
    toolchain.info("Static analysis for %s (%s, %s)" %
                   ('MBED', target.name, toolchain_name))

    # Common Headers
    toolchain.copy_files([MBED_HEADER], MBED_LIBRARIES)
    toolchain.copy_files(toolchain.scan_resources(MBED_DRIVERS).headers,
                         MBED_LIBRARIES)
    toolchain.copy_files(toolchain.scan_resources(MBED_PLATFORM).headers,
                         MBED_LIBRARIES)
    toolchain.copy_files(toolchain.scan_resources(MBED_HAL).headers,
                         MBED_LIBRARIES)

    # Target specific sources
    hal_src = join(MBED_TARGETS_PATH, "hal")
    hal_implementation = toolchain.scan_resources(hal_src)

    # Copy files before analysis
    toolchain.copy_files(hal_implementation.headers +
                         hal_implementation.hex_files, build_target,
                         resources=hal_implementation)
    incdirs = toolchain.scan_resources(build_target)

    target_includes = ["-I%s" % i for i in incdirs.inc_dirs]
    target_includes.append("-I%s"% str(build_target))
    target_includes.append("-I%s"% str(hal_src))
    target_c_sources = " ".join(incdirs.c_sources)
    target_cpp_sources = " ".join(incdirs.cpp_sources)
    target_macros = ["-D%s"% s for s in
                     toolchain.get_symbols() + toolchain.macros]

    # Common Sources
    mbed_resources = toolchain.scan_resources(MBED_COMMON)

    # Gather include paths, c, cpp sources and macros to transfer to cppcheck
    # command line
    mbed_includes = ["-I%s" % i for i in mbed_resources.inc_dirs]
    mbed_includes.append("-I%s"% str(build_target))
    mbed_includes.append("-I%s"% str(MBED_DRIVERS))
    mbed_includes.append("-I%s"% str(MBED_PLATFORM))
    mbed_includes.append("-I%s"% str(MBED_HAL))
    mbed_c_sources = " ".join(mbed_resources.c_sources)
    mbed_cpp_sources = " ".join(mbed_resources.cpp_sources)

    target_includes = [inc.strip() for inc in target_includes]
    mbed_includes = [inc.strip() for inc in mbed_includes]
    target_macros = [mac.strip() for mac in target_macros]

    check_cmd = cppcheck_cmd
    check_cmd += cppcheck_msg_format
    check_cmd += target_includes
    check_cmd += mbed_includes
    check_cmd += target_macros

    # We need to pass some parames via file to avoid "command line too long in
    # some OSs"
    tmp_file = tempfile.NamedTemporaryFile(delete=False)
    tmp_file.writelines(line + '\n' for line in target_c_sources.split())
    tmp_file.writelines(line + '\n' for line in target_cpp_sources.split())
    tmp_file.writelines(line + '\n' for line in mbed_c_sources.split())
    tmp_file.writelines(line + '\n' for line in mbed_cpp_sources.split())
    tmp_file.close()
    check_cmd += ["--file-list=%s"% tmp_file.name]

    _stdout, _stderr, _ = run_cmd_ext(check_cmd)
    if verbose:
        print _stdout
    print _stderr


def static_analysis_scan_lib(lib_id, target, toolchain, cppcheck_cmd,
                             cppcheck_msg_format, verbose=False,
                             clean=False, macros=None, notify=None, jobs=1,
                             extra_verbose=False, build_profile=None):
    """Perform static analysis on a library as if it were to be compiled for a
    particular target and toolchain combination
    """
    lib = Library(lib_id)
    if lib.is_supported(target, toolchain):
        static_analysis_scan_library(
            lib.source_dir, lib.build_dir, target, toolchain, cppcheck_cmd,
            cppcheck_msg_format, lib.dependencies, verbose=verbose,
            clean=clean, macros=macros, notify=notify, jobs=jobs,
            extra_verbose=extra_verbose, build_profile=build_profile)
    else:
        print('Library "%s" is not yet supported on target %s with toolchain %s'
              % (lib_id, target.name, toolchain))


def static_analysis_scan_library(src_paths, build_path, target, toolchain_name,
                                 cppcheck_cmd, cppcheck_msg_format,
                                 dependencies_paths=None,
                                 name=None, clean=False, notify=None,
                                 verbose=False, macros=None, jobs=1,
                                 extra_verbose=False, build_profile=None):
    """ Function scans library for statically detectable defects

    Positional arguments:
    src_paths - the list of library paths to scan
    build_path - the location directory of result files
    target - the target to fake the build for
    toolchain_name - pretend you would compile with this toolchain
    cppcheck_cmd - the command used to do static analysis
    cppcheck_msg_format - the format of the check messages

    Keyword arguments:
    dependencies_paths - the paths to sources that this library depends on
    name - the name of this library
    clean - start from a clean slate
    notify - the notification event handling function
    verbose - more printing!
    macros - extra macros to compile with
    jobs - number of commands to run at once
    extra_verbose - even moar printing
    build_profile - a dict of flags that will be passed to the compiler
    """
    if type(src_paths) != ListType:
        src_paths = [src_paths]

    for src_path in src_paths:
        if not exists(src_path):
            raise Exception("The library source folder does not exist: %s",
                            src_path)

    # Toolchain instance
    toolchain = TOOLCHAIN_CLASSES[toolchain_name](target, macros=macros,
                                                  notify=notify,
                                                  extra_verbose=extra_verbose,
                                                  build_profile=build_profile)
    toolchain.VERBOSE = verbose
    toolchain.jobs = jobs

    # The first path will give the name to the library
    name = basename(src_paths[0])
    toolchain.info("Static analysis for library %s (%s, %s)" %
                   (name.upper(), target.name, toolchain_name))

    # Scan Resources
    resources = []
    for src_path in src_paths:
        resources.append(toolchain.scan_resources(src_path))

    # Dependencies Include Paths
    dependencies_include_dir = []
    if dependencies_paths is not None:
        for path in dependencies_paths:
            lib_resources = toolchain.scan_resources(path)
            dependencies_include_dir.extend(lib_resources.inc_dirs)

    # Create the desired build directory structure
    bin_path = join(build_path, toolchain.obj_path)
    mkdir(bin_path)
    tmp_path = join(build_path, '.temp', toolchain.obj_path)
    mkdir(tmp_path)

    # Gather include paths, c, cpp sources and macros to transfer to cppcheck
    # command line
    includes = ["-I%s" % i for i in dependencies_include_dir + src_paths]
    c_sources = " "
    cpp_sources = " "
    macros = ['-D%s' % s for s in toolchain.get_symbols() + toolchain.macros]

    # Copy Headers
    for resource in resources:
        toolchain.copy_files(resource.headers, build_path, resources=resource)
        includes += ["-I%s" % i for i in resource.inc_dirs]
        c_sources += " ".join(resource.c_sources) + " "
        cpp_sources += " ".join(resource.cpp_sources) + " "

    dependencies_include_dir.extend(
        toolchain.scan_resources(build_path).inc_dirs)

    includes = [inc.strip() for inc in includes]
    macros = [mac.strip() for mac in macros]

    check_cmd = cppcheck_cmd
    check_cmd += cppcheck_msg_format
    check_cmd += includes
    check_cmd += macros

    # We need to pass some parameters via file to avoid "command line too long
    # in some OSs". A temporary file is created to store e.g. cppcheck list of
    # files for command line
    tmp_file = tempfile.NamedTemporaryFile(delete=False)
    tmp_file.writelines(line + '\n' for line in c_sources.split())
    tmp_file.writelines(line + '\n' for line in cpp_sources.split())
    tmp_file.close()
    check_cmd += ["--file-list=%s"% tmp_file.name]

    # This will allow us to grab result from both stdio and stderr outputs (so
    # we can show them) We assume static code analysis tool is outputting
    # defects on STDERR
    _stdout, _stderr, _ = run_cmd_ext(check_cmd)
    if verbose:
        print _stdout
    print _stderr


def print_build_results(result_list, build_name):
    """ Generate result string for build results

    Positional arguments:
    result_list - the list of results to print
    build_name - the name of the build we are printing result for
    """
    result = ""
    if len(result_list) > 0:
        result += build_name + "\n"
        result += "\n".join(["  * %s" % f for f in result_list])
        result += "\n"
    return result

def print_build_memory_usage(report):
    """ Generate result table with memory usage values for build results
    Aggregates (puts together) reports obtained from self.get_memory_summary()

    Positional arguments:
    report - Report generated during build procedure.
    """
    from prettytable import PrettyTable
    columns_text = ['name', 'target', 'toolchain']
    columns_int = ['static_ram', 'stack', 'heap', 'total_ram', 'total_flash']
    table = PrettyTable(columns_text + columns_int)

    for col in columns_text:
        table.align[col] = 'l'

    for col in columns_int:
        table.align[col] = 'r'

    for target in report:
        for toolchain in report[target]:
            for name in report[target][toolchain]:
                for dlist in report[target][toolchain][name]:
                    for dlistelem in dlist:
                        # Get 'memory_usage' record and build table with
                        # statistics
                        record = dlist[dlistelem]
                        if 'memory_usage' in record and record['memory_usage']:
                            # Note that summary should be in the last record of
                            # 'memory_usage' section. This is why we are
                            # grabbing last "[-1]" record.
                            row = [
                                record['description'],
                                record['target_name'],
                                record['toolchain_name'],
                                record['memory_usage'][-1]['summary'][
                                    'static_ram'],
                                record['memory_usage'][-1]['summary']['stack'],
                                record['memory_usage'][-1]['summary']['heap'],
                                record['memory_usage'][-1]['summary'][
                                    'total_ram'],
                                record['memory_usage'][-1]['summary'][
                                    'total_flash'],
                            ]
                            table.add_row(row)

    result = "Memory map breakdown for built projects (values in Bytes):\n"
    result += table.get_string(sortby='name')
    return result

def write_build_report(build_report, template_filename, filename):
    """Write a build report to disk using a template file

    Positional arguments:
    build_report - a report generated by the build system
    template_filename - a file that contains the template for the style of build
                        report
    filename - the location on disk to write the file to
    """
    build_report_failing = []
    build_report_passing = []

    for report in build_report:
        if len(report["failing"]) > 0:
            build_report_failing.append(report)
        else:
            build_report_passing.append(report)

    env = Environment(extensions=['jinja2.ext.with_'])
    env.loader = FileSystemLoader('ci_templates')
    template = env.get_template(template_filename)

    with open(filename, 'w+') as placeholder:
        placeholder.write(template.render(
            failing_builds=build_report_failing,
            passing_builds=build_report_passing))
