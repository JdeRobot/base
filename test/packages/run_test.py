import docker
import yaml
import sys




yamlFile = sys.argv[1]
client = docker.from_env()
images={}

result = open('test_result.txt', 'w')


## Build Image ##
def build_image(ipath, itag):
  try:
    print "Building " + image 
    client.images.build(pull=True, path=ipath, tag=itag, rm=True, stream=True)
  except docker.errors.BuildError as exc:
    return "ERROR " + exc.__str__()
  except docker.errors.APIError as exc:
    return "ERROR " + exc.__str__()

  return "OK" 


## remove Image ##
def remove_image(itag):
  try:
    print "Removing " + image 
    client.images.remove(image=itag, force=True)
  except docker.errors.BuildError as exc:
    pass
  except docker.errors.APIError as exc:
    print (exc)



## Open File ##
with open(yamlFile, 'r') as stream:
  try:
    images=yaml.load(stream)
  except yaml.YAMLError as exc:
    print(exc)


for image in images:
  if images[image]["run"]:
    
    sol = build_image(images[image]["path"], image)
    remove_image(image)

    result.write(image + ": " + sol + "\n")

result.close()
  
  
  
  
