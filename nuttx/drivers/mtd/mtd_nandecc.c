#include <nuttx/config.h>
#include <nuttx/mtd/nand_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/hamming.h>
#include <nuttx/mtd/nand_scheme.h>
#include <nuttx/mtd/nand_ecc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nandecc_readpage
 *
 * Description:
 *   Reads the data and/or spare areas of a page of a NAND FLASH chip and
 *   verifies that the data is valid using the ECC information contained in
 *   the spare area. If a buffer pointer is NULL, then the corresponding area
 *   is not saved. If extra pointer is NULL, 'extra data' is not saved.
 *   data should have size of 'pagesize' space
 *   extra should have size of 'scheme->nxbytes' space
 *
 * Input parameters:
 *   nand  - Upper-half, NAND FLASH interface
 *   block - Number of the block where the page to read resides.
 *   page  - Number of the page to read inside the given block.
 *   data  - Buffer where the data area will be stored.
 *   extra - Buffer where the extra data will be stored.
 *
 * Returned value.
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

int nandecc_readpage(FAR struct nand_dev_s *nand, off_t block,
                     unsigned int page, FAR void *data, FAR void *extra)
{
  FAR struct nand_raw_s *raw;
  FAR struct nand_model_s *model;
  FAR const struct nand_scheme_s *scheme;
  void *spare;
  unsigned int pagesize;
  unsigned int sparesize;
  int ret;
  int result;

  fvdbg("block=%d page=%d data=%p extra=%d\n", (int)block, page, data, extra);

  if (!data && !extra)
    return -EINVAL;

  /* Get convenience pointers */

  DEBUGASSERT(nand && nand->raw);
  raw   = nand->raw;
  model = &raw->model;

  /* Get size parameters */

  pagesize  = nandmodel_getpagesize(model);
  sparesize = nandmodel_getsparesize(model);
  scheme = nandmodel_getscheme(model);

  spare = raw->spare;
  memset(spare, 0xff, sparesize);

  ret = NAND_RAWREAD(nand->raw, block, page, data, spare);
  if (ret < 0)
    {
      fdbg("ERROR: Failed to read page:d\n", ret);
      return ret;
    }

  result = OK;

  if (data)
    {
      /* Retrieve ECC information from page */
      nandscheme_readecc(scheme, spare, raw->ecc);

      /* Use the ECC data to verify the page */

      ret = hamming_verify256x(data, pagesize, raw->ecc);
      if (ret)
        {
          if (ret == HAMMING_ERROR_SINGLEBIT)
            {
              result = 1;  // single bit err, corrected
            }
          else
            {
              fdbg("ERROR: Block=%d page=%d Unrecoverable error: %d\n",
                   block, page, ret);
              result = -EIO;  // multiple bit err
            }
        }
    }
    if (extra)
      nandscheme_readextra(scheme, spare, extra, scheme->nxbytes, 0);

  return result;
}

/****************************************************************************
 * Name: nandecc_writepage
 *
 * Description:
 *   Writes the data and/or extra data of a NAND FLASH page after calculating
 *   ECCs for the data area and storing them in the spare. If no data buffer
 *   is provided, only extra data will be saved. If no extra data  buffer is
 *   provided, the spare area is still written with the ECC information
 *   calculated on the data buffer.
 *
 *   Regard data is buf of 'pagesize' bytes and extra is buf of
 *   'scheme->nxbytes' bytes
 *
 * Input parameters:
 *   nand  - Upper-half, NAND FLASH interface
 *   block - Number of the block where the page to write resides.
 *   page  - Number of the page to write inside the given block.
 *   data  - Buffer containing the data to be writting
 *   extra - Buffer containing the extra data to be written.
 *
 * Returned value.
 *   OK is returned in success; a negated errno value is returned on failure.
 *
 ****************************************************************************/

int nandecc_writepage(FAR struct nand_dev_s *nand, off_t block,
                      unsigned int page,  FAR const void *data,
                      FAR void *extra)
{
  FAR struct nand_raw_s *raw;
  FAR struct nand_model_s *model;
  FAR const struct nand_scheme_s *scheme;
  void *spare;
  unsigned int pagesize;
  unsigned int sparesize;
  int ret;

  fvdbg("block=%d page=%d data=%p extra=%d\n", (int)block, page, data, extra);

  if (!extra && !data)
    return OK;

  /* Get convenience pointers */

  DEBUGASSERT(nand && nand->raw);
  raw   = nand->raw;
  model = &raw->model;

  /* Get size parameters */

  pagesize  = nandmodel_getpagesize(model);
  sparesize = nandmodel_getsparesize(model);
  scheme = nandmodel_getscheme(model);

  /* Store code in spare buffer, either the buffer provided by the caller or
   * the scatch buffer in the raw NAND structure.
   */
  spare = raw->spare;
  memset(spare, 0xff, sparesize);

  if (extra)
    nandscheme_writeextra(scheme, spare, extra, scheme->nxbytes, 0);

  /* Compute ECC on the new data, if provided */
  if (data)
    {
      /* Set hamming code set to 0xffff.. to keep existing bytes */
      memset(raw->ecc, 0xff, CONFIG_MTD_NAND_MAXSPAREECCBYTES);

      /* Compute hamming code on data */
      hamming_compute256x(data, pagesize, raw->ecc);

      /* Fill ECC */
      nandscheme_writeecc(scheme, spare, raw->ecc);
    }


  /* Perform page write operation */

  ret = NAND_RAWWRITE(nand->raw, block, page, data, spare);
  if (ret < 0)
    {
      fdbg("ERROR: Failed to write page:d\n", ret);
    }

  return ret;
}
