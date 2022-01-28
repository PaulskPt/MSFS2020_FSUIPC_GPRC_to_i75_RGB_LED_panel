

from Interstate75_GPRMC_64x32_matrix_code_v1 import main as my_start


def main():
    try:
        my_start()
    except KeyboardInterrupt:
        raise SystemExit
    
    
if __name__=="__main__":
    main()
