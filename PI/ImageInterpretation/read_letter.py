#from PIL import Image
#import pytesseract
#print(pytesseract.image_to_string(Image.open('./pictures/A.jpg')))
#image=Image.open('output.jpg')
#target = pytesseract.image_to_string(image, lang='eng', config='--psm 8 --oem 3')
#target=pytesseract.image_to_string(Image.open('words.jpg'), config='--psm 10 --oem 3 -c tessedit_char_whitelist=ABCDEF')
#print (target)

import pytesseract
from PIL import Image, ImageEnhance, ImageFilter

im = Image.open("output.jpg") # the second one 
im = im.filter(ImageFilter.MedianFilter())
enhancer = ImageEnhance.Contrast(im)
im = enhancer.enhance(2)
im = im.convert('1')
im.save('temp2.jpg')
text = pytesseract.image_to_string(Image.open('temp2.jpg'), config='--psm 10 --oem 3 -c tessedit_char_whitelist=ABCDEF')
print(text)