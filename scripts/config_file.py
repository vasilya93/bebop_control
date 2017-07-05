#!/usr/bin/python
import ConfigParser
import io

DEFAULT_CONFIG_FILENAME = "config.ini"

def writeConfigFile(configDir, dictionary, configFilename = None):
    if configFilename == None:
        configFilename = DEFAULT_CONFIG_FILENAME
    configPath = configDir + "/" + configFilename

    configFile = open(configPath, 'w')
    config = ConfigParser.ConfigParser()
    for key in dictionary:
        config.add_section(key)
        for subKey in dictionary[key]:
            config.set(key, subKey, dictionary[key][subKey])
    config.write(configFile)
    configFile.close()

def readConfigFile(configDir, configFilename = None):
    if configFilename == None:
        configFilename = DEFAULT_CONFIG_FILENAME
    configPath = configDir + "/" + configFilename

    with open(configPath) as f:
        configContent = f.read()
    config = ConfigParser.RawConfigParser(allow_no_value=True)
    config.readfp(io.BytesIO(configContent))

    dictionary = {}
    for section in config.sections():
        options = {}
        for option in config.options(section):
            option_value = config.get(section, option)
            options[option] = option_value
        dictionary[section] = options
    return dictionary

def readConfigSection(configDir, configSection, configFilename = None):
    dictionary = readConfigFile(configDir, configFilename)
    return dictionary[configSection]

def readConfigProperty(configDir, configSection, configKey, configFilename = None):
    dictionary = readConfigFile(configDir, configFilename)
    return dictionary[configSection][configKey]
