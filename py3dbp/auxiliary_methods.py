from decimal import Decimal
from .constants import Axis
DEFAULT_NUMBER_OF_DECIMALS = 3
START_POSITION = [0, 0, 0]

def rect_intersect(item1, item2, x, y):
    """Estimate whether two items get intersection in one dimension.
    Args:
        item1, item2: any two items in item list.
        x,y: Axis.LENGTH/ Axis.Height/ Axis.WIDTH.
    Returns:
        Boolean variable: False when two items get intersection in one dimension; True when two items do not intersect in one dimension.
    """
    
    d1 = item1.get_dimension() 
    d2 = item2.get_dimension() 
    
    cx1 = item1.position[x] + d1[x]/2 
    cy1 = item1.position[y] + d1[y]/2
    cx2 = item2.position[x] + d2[x]/2 
    cy2 = item2.position[y] + d2[y]/2
    
    ix = max(cx1, cx2) - min(cx1, cx2) # ix: |cx1-cx2|
    iy = max(cy1, cy2) - min(cy1, cy2) # iy: |cy1-cy2|
    
    return ix < (d1[x] + d2[x])/2 and iy < (d1[y] + d2[y])/2 


def intersect(item1, item2):
    """Estimate whether two items get intersection in 3D dimension.
    Args:
        item1, item2: any two items in item list.
    Returns:
        Boolean variable: False when two items get intersection; True when two items do not intersect.
    """
    
    return ( 
    rect_intersect(item1, item2, Axis.LENGTH, Axis.HEIGHT) and # xz dimension
    rect_intersect(item1, item2, Axis.HEIGHT, Axis.WIDTH) and # yz dimension
    rect_intersect(item1, item2, Axis.LENGTH, Axis.WIDTH)) # xy dimension


def get_limit_number_of_decimals(number_of_decimals):
    return Decimal('1.{}'.format('0' * number_of_decimals))


def set_to_decimal(value, number_of_decimals):
    number_of_decimals = get_limit_number_of_decimals(number_of_decimals)

    return Decimal(value).quantize(number_of_decimals)
